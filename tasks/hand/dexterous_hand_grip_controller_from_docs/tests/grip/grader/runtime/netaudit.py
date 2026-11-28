"""The network audit of a program executed at verify (grader side; the standard library and contract.names only).

Verify runs with open egress (engine), so a program the grader executes could download the public arm or fetch material a
background process pushed out before the lock. Three checks, all feeding the contract gate:
  * static (`scan_imports`): the submission's own .py files (files byte-identical to a shipped library file exempt,
    `allow_digests`) import no networking module (`NETWORK_MODULES`, incl. `multiprocessing.connection`, `logging.handlers`,
    `xmlrpc`, `ctypes`), load no module under a computed name (`__import__` / `importlib.import_module` /
    `importlib.util.find_spec` with a non-literal argument), execute no computed code (`exec` / `eval` / `compile` with a
    non-literal argument) and start no other program (`subprocess`, `pty`; and `process_call_hits`: the names of
    PROCESS_CALLS as what they are, attributes of the modules of PROCESS_CALL_MODULES under any name bound to the
    module, through getattr / vars / __dict__ with the name as a string, or imported from such a module; a function of
    the controller's own with such a name is no hit); every hit is listed. The names and modules are contract.names'
    AUDIT_* tuples (one source: the controller interface prints them). It fails closed: a .py file that is not a
    regular file, is larger than the per-file cap, cannot be read, declares an encoding other than UTF-8, ASCII or
    Latin-1 (decided from its first two lines, before anything is decoded with it: `source_text`), does not decode, or
    that the parser refuses (SyntaxError, ValueError, RecursionError, MemoryError) is a hit; symbolic links are skipped
    (the program's copy never holds them). `audit` (with `controller_class`) is the whole static check of a controller folder;
    `audit_in_child` runs it in a child process under its own address-space, CPU and wall-clock limits, so the
    grader's own process reads no byte of the sources (the child imports this module from the folders that hold the
    packages contract and grader), and a child without a result is a failed check; `tree_shape`
    is the walk that only stats (entries, depth, bytes) and runs before anything is copied or read
  * runtime, namespace-wide (`NetCounters`): the container network namespace's counters of outbound activity (TCP active
    opens, UDP / UDP-Lite datagrams sent, ICMP echo requests sent; v4 and v6: /proc/net/snmp, /proc/net/snmp6) read before
    and after the units run, published as the statistic `net_outbound_events`, which JUDGES NOTHING: the counters are
    the whole namespace's (loopback, the image's own services and every other process of the sandbox count; every
    validate with the units run read 0 or 2 with the refusal in force), so they cannot tell the controller's traffic from
    the sandbox's. The network gate rests on the client's refusal (client._no_network; fail-closed as root) and on the
    runner account's own sockets (verify_main.network_gate).
  * runtime, per process (`SocketSampler`): the remote inet sockets (tcp / udp, v4 / v6, neither loopback nor unspecified)
    of every process of the runner account, sampled every `period_s`; any sample > 0 zeroes the gate (stat
    `remote_sockets_seen`). With `oom_floor` (run as root) each pass also KILLS (SIGKILL) any of these processes whose
    `oom_score_adj` reads below that floor and counts the kills (`oom_killed`; a record only). Every process of the
    program is started at the floor (`choom`, which writes the value and then execs the interpreter, same pid) and a
    legitimate program never lowers it; a process found below the floor has lowered its own value, so it is stopped (it
    then ends as a crash under the unit's rules: a dead controller) rather than raised. The launch chain's own helpers
    (`runuser`, `prlimit`, `choom`), which run as the account for a few milliseconds at the inherited value before
    `choom` writes the floor, are told apart by their executable name and are never killed, so an ordinary launch is
    never stopped.
"""

import ast
import builtins
import codecs
import hashlib
import io
import json
import os
import shutil
import signal
import stat
import subprocess
import sys
import threading
import time
import tokenize

from contract import names as _N

__all__ = ["NETWORK_MODULES", "PROCESS_CALLS", "scan_imports", "SocketSampler", "NetCounters", "net_counters", "count_remote_sockets", "uid_pids",
           "source_text", "controller_class", "audit", "audit_in_child", "tree_shape",
           "process_call_hits", "PROCESS_CALL_MODULES", "LAUNCH_CHAIN_COMMS", "code_call_hits", "LITERAL_IMPORT_CALLS",
           "IMPORT_MODULES", "BUILTINS_MODULES"]

# the audit's names, one source (contract.names; the controller interface prints them): the refused modules; the calls
# that start another program, as attributes of PROCESS_CALL_MODULES or imported from them; the calls that load a module
# under a computed name, as the built-in __import__ or attributes of IMPORT_MODULES (and of every module inside it),
# LITERAL_IMPORT_CALLS of them only with a literal that names a refused module; the built-ins that run computed code,
# by their plain name where it is the built-in or as attributes of BUILTINS_MODULES
NETWORK_MODULES = _N.AUDIT_MODULES
PROCESS_CALLS = _N.AUDIT_PROCESS_CALLS
PROCESS_CALL_MODULES = _N.AUDIT_PROCESS_MODULES
DYNAMIC_IMPORT_CALLS = _N.AUDIT_IMPORT_CALLS
LITERAL_IMPORT_CALLS = _N.AUDIT_LITERAL_IMPORT_CALLS
IMPORT_MODULES = _N.AUDIT_IMPORT_MODULES
CODE_CALLS = _N.AUDIT_CODE_CALLS
BUILTINS_MODULES = _N.AUDIT_BUILTINS_MODULES
# the names of the code calls and import calls that are built-ins (exec, eval, compile, __import__)
BUILTIN_CALLS = tuple(n for n in CODE_CALLS + DYNAMIC_IMPORT_CALLS if hasattr(builtins, n))
PATH_DEPTH_MAX = 200                # attributes, calls and subscripts followed from a name (code_call_hits)
VALUE_ROUNDS_MAX = 20               # rounds of assignments followed from one name to the next (code_call_hits)
PATHS_MAX = 32                      # paths kept for one name or expression (code_call_hits)
# the modules a path is followed from (code_call_hits): those of the code and import calls, and sys (sys.modules)
_ROOTS = BUILTINS_MODULES + IMPORT_MODULES + ("sys",)


def _kept(paths):
    """The paths code_call_hits follows: those that start at a module of _ROOTS and have at most PATH_DEPTH_MAX parts,
    at most PATHS_MAX of them (the first in sorted order), so that no source makes the audit's work grow without bound."""
    out = sorted(p for p in paths if p.split(".")[0] in _ROOTS and p.count(".") < PATH_DEPTH_MAX)
    return set(out[:PATHS_MAX])
# the source encodings a .py may declare (codecs' names; utf-8-sig = UTF-8 with its byte-order mark)
ALLOWED_ENCODINGS = ("utf-8", "utf-8-sig", "ascii", "iso8859-1")
# what a parser may raise on hostile source (Python 3.12: past its nesting limits RecursionError or MemoryError)
PARSE_ERRORS = (SyntaxError, ValueError, RecursionError, MemoryError)
AUDIT_AS_MB = 4096                  # the address space of the static checks' child process
HIT_TEXT_MAX = 200                  # characters of one audit hit as the child reports it
HITS_MAX = 200                      # hits the child reports in full (their count always)


def _hit(name):
    name = name or ""
    parts = name.split(".")
    return any(".".join(parts[:k]) in NETWORK_MODULES for k in range(1, len(parts) + 1))


def _literal(node):
    return isinstance(node, ast.Constant) and isinstance(node.value, str)


def _module_of(node, alias):
    """The module of PROCESS_CALL_MODULES that the expression `node` stands for, or None: a name bound to it (`alias`),
    an attribute of that name (`shutil.os`), `sys.modules["os"]`, `__import__("os")` or
    `importlib.import_module("os")`."""
    if isinstance(node, ast.Name):
        return alias.get(node.id)
    if isinstance(node, ast.Attribute) and node.attr in PROCESS_CALL_MODULES:
        return node.attr
    if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Attribute) and node.value.attr == "modules" \
            and _literal(node.slice) and node.slice.value in PROCESS_CALL_MODULES:
        return node.slice.value
    if isinstance(node, ast.Call) and node.args and _literal(node.args[0]) and \
            node.args[0].value in PROCESS_CALL_MODULES:
        f = node.func
        fname = f.id if isinstance(f, ast.Name) else (f.attr if isinstance(f, ast.Attribute) else "")
        if fname in ("__import__", "import_module"):
            return node.args[0].value
    return None


def _module_aliases(tree):
    """{name: module} of the names a source binds to a module of PROCESS_CALL_MODULES (anywhere in the file): the
    module's own name, `import os [as o]` (`import os.path` binds os), `from x import os [as o]`, and assignments of
    such an expression (`o = os`, `o = sys.modules["os"]`, chains of them), to a fixed point."""
    alias = {m: m for m in PROCESS_CALL_MODULES}
    binds = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            for a in n.names:
                if a.asname is None and a.name.split(".")[0] in PROCESS_CALL_MODULES:
                    alias[a.name.split(".")[0]] = a.name.split(".")[0]
                elif a.asname is not None and a.name in PROCESS_CALL_MODULES:
                    alias[a.asname] = a.name
        elif isinstance(n, ast.ImportFrom):
            for a in n.names:
                if a.name in PROCESS_CALL_MODULES:
                    alias[a.asname or a.name] = a.name
        elif isinstance(n, ast.Assign):
            binds += [(t, n.value) for t in n.targets]
        elif isinstance(n, (ast.AnnAssign, ast.NamedExpr)) and n.value is not None:
            binds.append((n.target, n.value))
    changed = True
    while changed:
        changed = False
        for target, value in binds:
            m = _module_of(value, alias)
            if m and isinstance(target, ast.Name) and alias.get(target.id) != m:
                alias[target.id] = m
                changed = True
    return alias


def process_call_hits(tree):
    """[reason] of the references to a call that starts another program: a name of PROCESS_CALLS as an attribute of a
    module of PROCESS_CALL_MODULES (under any name bound to it), through getattr(module, "name"), vars(module)["name"]
    or module.__dict__["name"], or imported from such a module (`from os import system`, `from os import *`). A
    function, method or attribute of the controller's own that merely has such a name is no hit."""
    alias = _module_aliases(tree)
    out = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Attribute) and n.attr in PROCESS_CALLS:
            m = _module_of(n.value, alias)
            if m:
                out.append("%s.%s (starts a program)" % (m, n.attr))
        elif isinstance(n, ast.ImportFrom) and n.level == 0 and n.module in PROCESS_CALL_MODULES:
            for a in n.names:
                if a.name == "*":
                    out.append("from %s import * (imports the calls that start a program)" % n.module)
                elif a.name in PROCESS_CALLS:
                    out.append("%s.%s (starts a program)" % (n.module, a.name))
        elif isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "getattr" and len(n.args) >= 2:
            m = _module_of(n.args[0], alias)
            if m and _literal(n.args[1]) and n.args[1].value in PROCESS_CALLS:
                out.append("%s.%s (starts a program)" % (m, n.args[1].value))
        elif isinstance(n, ast.Subscript) and _literal(n.slice) and n.slice.value in PROCESS_CALLS:
            v, m = n.value, None
            if isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "vars" and v.args:
                m = _module_of(v.args[0], alias)
            elif isinstance(v, ast.Attribute) and v.attr == "__dict__":
                m = _module_of(v.value, alias)
            if m:
                out.append("%s.%s (starts a program)" % (m, n.slice.value))
    return out


# ------------------------------------------------------------------------------------- the code calls and import calls
class _Scope(object):
    """A scope of a source for code_call_hits: "module", "function" (a def, async def or lambda), "class" or "comp" (a
    comprehension). `bound`: the names it surely binds (a function or comprehension: every name it binds anywhere, as
    Python makes them local; the module: a def, class, import or assignment written as a statement of its own top
    level); `maybe`: every name it binds in any way; `deleted`: names deleted at module level; `values`: {name:
    [a dotted path (an import) or an expression (an assignment)]}; `glob` / `nonloc`: the global / nonlocal names."""

    def __init__(self, kind, parent, registry=None):
        self.kind, self.parent = kind, parent
        if registry is not None:
            registry.append(self)
        self.bound, self.maybe, self.deleted = set(), set(), set()
        self.glob, self.nonloc = set(), set()
        self.values = {}
        self.stars = []


class _CodeCalls(object):
    """The references of a source to the built-ins exec / eval / compile / __import__ and to the import calls of
    IMPORT_MODULES, by Python's scoping (simplified: class bodies bind nothing; at module level only a binding written
    as a top-level statement shadows a built-in)."""

    def __init__(self, tree):
        self.scope_of = {}
        self.parent = {}
        for n in ast.walk(tree):
            for c in ast.iter_child_nodes(n):
                self.parent[c] = n
        self._scopes = []
        self.module = _Scope("module", None, self._scopes)
        for st in tree.body:
            self._stmt(st, self.module, True)
        # the paths each bound name may stand for, from its imports, then from its assignments to a fixed point
        self.table = {}
        assigns = []
        for sc in self._scopes:
            for name, vals in sc.values.items():
                self.table[(id(sc), name)] = _kept(v for v in vals if isinstance(v, str))
                assigns += [(sc, name, v) for v in vals if not isinstance(v, str)]
        for _round in range(VALUE_ROUNDS_MAX):
            changed = False
            for sc, name, v in assigns:
                key = (id(sc), name)
                new = _kept(self.table[key] | self.path(v, self.scope_of.get(v, sc)))
                if new != self.table[key]:
                    self.table[key], changed = new, True
            if not changed:
                break

    # -------------------------------------------------------------------------------------------------- bindings
    def _function_scope(self, sc):
        while sc.kind == "comp":
            sc = sc.parent
        return sc

    def _bind(self, sc, name, value, sure):
        if sc.kind in ("function", "comp"):
            if name in sc.glob:
                sc, sure = self.module, False
            elif name in sc.nonloc:
                up = sc.parent
                while up is not None and up.kind not in ("function", "comp"):
                    up = up.parent
                sc, sure = (up or self.module), False
            else:
                sc.bound.add(name)
        if sc.kind == "module":
            if sure:
                sc.bound.add(name)
        sc.maybe.add(name)
        if value is not None:
            sc.values.setdefault(name, []).append(value)

    def _targets(self, t, sc, value, sure):
        if isinstance(t, ast.Name):
            self._bind(sc, t.id, value, sure)
        elif isinstance(t, (ast.Tuple, ast.List)):
            for e in t.elts:
                self._targets(e, sc, None, sure)
        elif isinstance(t, ast.Starred):
            self._targets(t.value, sc, None, sure)
        else:
            self._expr(t, sc)

    def _pattern(self, p, sc):
        for n in ast.walk(p):
            for name in (getattr(n, "name", None), getattr(n, "rest", None)):
                if isinstance(name, str):
                    self._bind(sc, name, None, False)
            if isinstance(n, ast.MatchValue):
                self._expr(n.value, sc)

    def _args(self, a, sc_outer, sc_inner):
        for d in list(a.defaults) + [d for d in a.kw_defaults if d is not None]:
            self._expr(d, sc_outer)
        for x in a.posonlyargs + a.args + a.kwonlyargs + [v for v in (a.vararg, a.kwarg) if v is not None]:
            if x.annotation is not None:
                self._expr(x.annotation, sc_outer)
            self._bind(sc_inner, x.arg, None, True)

    def _stmt(self, st, sc, top):
        """One statement in scope `sc`; `top`: a statement of the module's own top level."""
        if isinstance(st, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for d in st.decorator_list:
                self._expr(d, sc)
            if st.returns is not None:
                self._expr(st.returns, sc)
            inner = _Scope("function", sc, self._scopes)
            self._args(st.args, sc, inner)
            self._bind(sc, st.name, None, top)
            for b in st.body:
                self._stmt(b, inner, False)
        elif isinstance(st, ast.ClassDef):
            for e in st.decorator_list + st.bases + [k.value for k in st.keywords]:
                self._expr(e, sc)
            self._bind(sc, st.name, None, top)
            inner = _Scope("class", sc, self._scopes)
            for b in st.body:
                self._stmt(b, inner, False)
        elif isinstance(st, ast.Import):
            for a in st.names:
                if a.asname:
                    self._bind(sc, a.asname, a.name, top)
                else:
                    self._bind(sc, a.name.split(".")[0], a.name.split(".")[0], top)
        elif isinstance(st, ast.ImportFrom):
            for a in st.names:
                if a.name == "*":
                    if st.level == 0 and st.module:
                        self.module.stars.append(st.module)
                elif st.level == 0 and st.module:
                    self._bind(sc, a.asname or a.name, st.module + "." + a.name, top)
                else:
                    self._bind(sc, a.asname or a.name, None, top)
        elif isinstance(st, (ast.Global, ast.Nonlocal)):
            (sc.glob if isinstance(st, ast.Global) else sc.nonloc).update(st.names)
        elif isinstance(st, ast.Assign):
            self._expr(st.value, sc)
            for t in st.targets:
                self._targets(t, sc, st.value if isinstance(t, ast.Name) else None, top)
        elif isinstance(st, ast.AnnAssign):
            self._expr(st.annotation, sc)
            if st.value is not None:
                self._expr(st.value, sc)
            self._targets(st.target, sc, st.value, top and st.value is not None)
        elif isinstance(st, ast.AugAssign):
            self._expr(st.value, sc)
            self._targets(st.target, sc, None, top)
        elif isinstance(st, ast.Delete):
            for t in st.targets:
                if isinstance(t, ast.Name):
                    if sc.kind in ("function", "comp") and t.id not in sc.glob:
                        sc.bound.add(t.id)
                        sc.maybe.add(t.id)
                    else:
                        self.module.deleted.add(t.id)
                else:
                    self._expr(t, sc)
        elif isinstance(st, (ast.For, ast.AsyncFor)):
            self._expr(st.iter, sc)
            self._targets(st.target, sc, None, False)
            for b in st.body + st.orelse:
                self._stmt(b, sc, False)
        elif isinstance(st, (ast.With, ast.AsyncWith)):
            for it in st.items:
                self._expr(it.context_expr, sc)
                if it.optional_vars is not None:
                    self._targets(it.optional_vars, sc, None, False)
            for b in st.body:
                self._stmt(b, sc, False)
        elif isinstance(st, ast.Match):
            self._expr(st.subject, sc)
            for c in st.cases:
                self._pattern(c.pattern, sc)
                if c.guard is not None:
                    self._expr(c.guard, sc)
                for b in c.body:
                    self._stmt(b, sc, False)
        elif isinstance(st, ast.stmt) and any(isinstance(getattr(st, f, None), list) for f in ("body", "handlers")):
            # if / while / try (and try*): their expressions, handlers' names, and nested statements (never top level)
            for f, v in ast.iter_fields(st):
                if isinstance(v, ast.expr):
                    self._expr(v, sc)
                elif isinstance(v, list):
                    for x in v:
                        if isinstance(x, ast.ExceptHandler):
                            if x.type is not None:
                                self._expr(x.type, sc)
                            if x.name:
                                self._bind(sc, x.name, None, False)
                            for b in x.body:
                                self._stmt(b, sc, False)
                        elif isinstance(x, ast.stmt):
                            self._stmt(x, sc, False)
                        elif isinstance(x, ast.expr):
                            self._expr(x, sc)
        else:
            for c in ast.iter_child_nodes(st):
                if isinstance(c, ast.expr):
                    self._expr(c, sc)
                elif isinstance(c, ast.stmt):
                    self._stmt(c, sc, False)

    def _expr(self, e0, sc0):
        """Every expression node under e0 gets its scope (iteratively: a long chain of operators is legal source)."""
        todo = [(e0, sc0)]
        while todo:
            e, sc = todo.pop()
            if isinstance(e, ast.Lambda):
                inner = _Scope("function", sc, self._scopes)
                self._args(e.args, sc, inner)
                todo.append((e.body, inner))
                continue
            if isinstance(e, (ast.ListComp, ast.SetComp, ast.GeneratorExp, ast.DictComp)):
                inner = _Scope("comp", sc, self._scopes)
                for k, g in enumerate(e.generators):
                    todo.append((g.iter, sc if k == 0 else inner))
                    self._targets(g.target, inner, None, True)
                    todo += [(c, inner) for c in g.ifs]
                todo += [(part, inner) for part in ((e.key, e.value) if isinstance(e, ast.DictComp) else (e.elt,))]
                continue
            if isinstance(e, ast.NamedExpr):
                todo.append((e.value, sc))
                self._bind(self._function_scope(sc), e.target.id, e.value, False)
                continue
            self.scope_of[e] = sc
            for c in ast.iter_child_nodes(e):
                if isinstance(c, ast.expr):
                    todo.append((c, sc))
                elif isinstance(c, ast.keyword):
                    todo.append((c.value, sc))

    # ----------------------------------------------------------------------------------------------- resolution
    def _values(self, sc, name):
        return set(getattr(self, "table", {}).get((id(sc), name), ()))

    def name_paths(self, name, sc):
        """The dotted paths the plain name `name`, used in scope `sc`, may stand for ("builtins.exec" when it is the
        built-in; the controller's own name with no such value: none)."""
        paths, s = set(), sc
        while s is not None:
            if s.kind in ("function", "comp"):
                if name in s.glob:
                    s = self.module
                    continue
                if name in s.nonloc:
                    s = s.parent
                    continue
                if name in s.bound:
                    return paths | self._values(s, name)
                s = s.parent
            elif s.kind == "class":
                if s is sc and name in s.maybe:
                    paths |= self._values(s, name)
                s = s.parent
            else:
                if name in s.maybe:
                    paths |= self._values(s, name)
                if name in s.bound and name not in s.deleted:
                    return paths
                break
        if name in BUILTIN_CALLS:
            paths.add(BUILTINS_MODULES[0] + "." + name)
        elif name == "__builtins__":
            paths.add(BUILTINS_MODULES[0])
        for m in self.module.stars:
            paths.add(m + "." + name)
        return _kept(paths)

    def path(self, e, sc, depth=0):
        """The dotted paths an expression may stand for: a module, or an attribute of one ("importlib.util.find_spec"),
        "()" marking a call's result; reached through names, imports, assignments, attributes, getattr / vars /
        __dict__ with a literal name, sys.modules["m"], __import__("m") and import_module("m"). Nested deeper than
        PATH_DEPTH_MAX it stands for nothing (no honest source reaches a module through such a chain)."""
        if depth > PATH_DEPTH_MAX:
            return set()
        d = depth + 1
        if isinstance(e, ast.Name):
            return self.name_paths(e.id, sc)
        if isinstance(e, ast.Attribute):
            return _kept(p + "." + e.attr for p in self.path(e.value, sc, d))
        if isinstance(e, ast.Call):
            fp = self.path(e.func, sc, d)
            out = {p + "()" for p in fp}
            arg0 = e.args[0] if e.args else None
            if arg0 is not None and _literal(arg0):
                for p in fp:
                    kind = _reference_kind(p)
                    if kind == ("import", "__import__"):
                        out.add(arg0.value.split(".")[0])
                    elif kind == ("import", "import_module"):
                        out.add(arg0.value)
            if isinstance(e.func, ast.Name) and e.func.id == "getattr" and len(e.args) >= 2 and _literal(e.args[1]):
                out |= {p + "." + e.args[1].value for p in self.path(e.args[0], sc, d)}
            return _kept(out)
        if isinstance(e, ast.Subscript) and _literal(e.slice):
            k, v = e.slice.value, e.value
            if isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "vars" and v.args:
                return _kept(p + "." + k for p in self.path(v.args[0], sc, d))
            out = set()
            for p in self.path(v, sc, d):
                if p.endswith(".__dict__"):
                    out.add(p[:-len(".__dict__")] + "." + k)
                elif p == "sys.modules":
                    out.add(k)
                elif p in BUILTINS_MODULES:
                    out.add(p + "." + k)
            return _kept(out)
        return set()

    def hits(self):
        out = []
        for e, sc in self.scope_of.items():
            if not isinstance(e, (ast.Name, ast.Attribute, ast.Call, ast.Subscript)):
                continue
            if not isinstance(getattr(e, "ctx", ast.Load()), ast.Load):
                continue
            kinds = sorted({k for k in (_reference_kind(p) for p in self.path(e, sc)) if k})
            for kind, name in kinds:
                why = self._judge(e, kind, name)
                if why:
                    out.append(why)
        return out

    def _judge(self, e, kind, name):
        up = self.parent.get(e)
        if isinstance(up, ast.Call) and up.func is e:
            arg0 = up.args[0] if up.args else None
            if arg0 is not None and _literal(arg0):
                return arg0.value if kind == "import" and _hit(arg0.value) else None
        elif isinstance(up, (ast.Assign, ast.AnnAssign, ast.NamedExpr)) and up.value is e and all(
                isinstance(t, ast.Name) for t in (up.targets if isinstance(up, ast.Assign) else [up.target])):
            return None                                     # an alias: the uses of its name are judged
        if kind == "code":
            return "%s(<computed code>)" % name
        if name in LITERAL_IMPORT_CALLS:
            return None
        return "%s(<computed name>)" % name


def _reference_kind(p):
    """("code", name) when the dotted path p is a built-in of CODE_CALLS, ("import", name) when it is the built-in
    __import__ or an import call reached from IMPORT_MODULES, else None."""
    parts = p.split(".")
    if len(parts) < 2:
        return None
    last = parts[-1]
    if parts[0] in BUILTINS_MODULES and len(parts) == 2:
        if last in CODE_CALLS:
            return ("code", last)
        if last in DYNAMIC_IMPORT_CALLS:
            return ("import", last)
    if parts[0] in IMPORT_MODULES and last in DYNAMIC_IMPORT_CALLS:
        return ("import", last)
    return None


def code_call_hits(tree):
    """[reason] of the references to computed code and to an import under a computed name: exec / eval / compile as
    the built-ins (the plain name where the file does not bind it itself, an attribute of BUILTINS_MODULES under any
    name bound to it or of __builtins__, through getattr / vars / __dict__, a name imported or assigned from one of
    these), and the import calls (the built-in __import__; the names of DYNAMIC_IMPORT_CALLS as attributes of
    IMPORT_MODULES or of a module or object reached from it, under any name bound to it, or imported from it). Called
    with a first argument that is not a plain string literal (or none), or used as anything but a call or the value of
    an assignment to a name, such a reference is a hit "<name>(<computed code>)" / "<name>(<computed name>)" (not for
    LITERAL_IMPORT_CALLS); an import call with a string literal that names a refused module is a hit of that module. A
    function, method or attribute of the controller's own that only has such a name is no hit, nor is another module's
    function of that name (from re import compile; re.compile(p))."""
    return _CodeCalls(tree).hits()


def source_text(data):
    """(text, None): a .py file's bytes decoded as Python imports them (its BOM and coding declaration), or (None, why)
    when it declares an encoding other than UTF-8, ASCII or Latin-1 or its bytes do not decode."""
    try:
        enc, _lines = tokenize.detect_encoding(io.BytesIO(data).readline)
        name = codecs.lookup(enc).name
    except (SyntaxError, LookupError, ValueError):
        return None, "an encoding declaration Python refuses"
    if name not in ALLOWED_ENCODINGS:
        return None, "declares the encoding %s (only UTF-8, ASCII or Latin-1)" % name[:40]
    try:
        return data.decode(name), None
    except (UnicodeDecodeError, LookupError):
        return None, "is not valid %s" % name


def _read_source(p, max_file_bytes):
    """(text, None) of a regular .py file, or (None, why): not a regular file, larger than the cap, unreadable, a
    refused encoding. A symbolic link gives (None, None): it is skipped (links never reach the program's copy)."""
    try:
        st = os.lstat(p)
    except OSError:
        return None, "cannot be inspected"
    if stat.S_ISLNK(st.st_mode):
        return None, None
    if not stat.S_ISREG(st.st_mode):
        return None, "is not a regular file"
    if max_file_bytes is not None and st.st_size > max_file_bytes:
        return None, "is larger than %d MB (a source file's cap)" % (max_file_bytes >> 20)
    try:
        with open(p, "rb") as fh:
            data = fh.read() if max_file_bytes is None else fh.read(max_file_bytes + 1)
    except OSError:
        return None, "cannot be read"
    if max_file_bytes is not None and len(data) > max_file_bytes:
        return None, "is larger than %d MB (a source file's cap)" % (max_file_bytes >> 20)
    text, why = source_text(data)
    return (text, why) if why else (data, None)


def scan_imports(root, allow_digests=(), max_file_bytes=None):
    """[(relpath, module or reason)] of networking imports in the .py files under `root` (symbolic links skipped), and
    of every .py file the audit cannot read: not a regular file, above `max_file_bytes`, unreadable, an encoding other
    than UTF-8 / ASCII / Latin-1, or source the parser refuses (SyntaxError, ValueError, RecursionError,
    MemoryError): such a file is a hit, never skipped."""
    allow = set(allow_digests or ())
    out = []
    for dp, dns, fns in os.walk(root):
        dns[:] = [d for d in dns if d != "__pycache__"]
        for fn in sorted(fns):
            if not fn.endswith(".py"):
                continue
            p = os.path.join(dp, fn)
            rel = os.path.relpath(p, root)
            data, why = _read_source(p, max_file_bytes)
            if why:
                out.append((rel, why))
                continue
            if data is None:
                continue
            if hashlib.sha256(data).hexdigest() in allow:
                continue
            try:
                tree = ast.parse(source_text(data)[0])
            except PARSE_ERRORS as e:
                out.append((rel, "does not parse (%s)" % type(e).__name__))
                continue
            for n in ast.walk(tree):
                if isinstance(n, ast.Import):
                    out += [(rel, a.name) for a in n.names if _hit(a.name)]
                elif isinstance(n, ast.ImportFrom):
                    if n.level == 0 and _hit(n.module):
                        out.append((rel, n.module))
                    elif n.level == 0 and n.module and any(_hit(n.module + "." + a.name) for a in n.names):
                        out.append((rel, n.module + "." + ",".join(a.name for a in n.names)))
            out += [(rel, why) for why in code_call_hits(tree)]
            out += [(rel, why) for why in process_call_hits(tree)]
    return out


def controller_class(path, max_file_bytes=None):
    """(True, None) when `path` is a regular file whose source (read as scan_imports reads it) parses and defines a class
    named Controller; else (False, why)."""
    data, why = _read_source(path, max_file_bytes)
    if why or data is None:
        return False, "controller.py %s" % (why or "is a symbolic link")
    try:
        tree = ast.parse(source_text(data)[0])
    except PARSE_ERRORS as e:
        return False, "controller.py does not parse (%s)" % type(e).__name__
    if any(isinstance(n, ast.ClassDef) and n.name == "Controller" for n in ast.walk(tree)):
        return True, None
    return False, "controller.py defines no class Controller"


def audit(folder, max_file_bytes):
    """The static checks of a controller folder (run in the child of audit_in_child, never in the grader's process):
    {"controller_file", "controller_error", "hits" (at most HITS_MAX, each text cut to HIT_TEXT_MAX), "n_hits"}."""
    ok, why = controller_class(os.path.join(folder, "controller.py"), max_file_bytes)
    hits = scan_imports(folder, max_file_bytes=max_file_bytes)
    return {"format": "grip-audit/1", "controller_file": bool(ok), "controller_error": why,
            "hits": [[str(a)[:HIT_TEXT_MAX], str(b)[:HIT_TEXT_MAX]] for a, b in hits[:HITS_MAX]],
            "n_hits": len(hits)}


# the static checks' child: python -I -B -S -c CHILD_BOOT <n> <n folders> --audit ...: the folders that hold the
# packages contract and grader first on its path (nothing else but the standard library), then this module's _main
CHILD_BOOT = ("import sys; n = int(sys.argv[1]); sys.path[:0] = sys.argv[2:2 + n]; "
              "from grader.runtime import netaudit; sys.exit(netaudit._main(sys.argv[2 + n:]))")


def _package_roots():
    """The folders that hold the packages contract and grader (one folder in the published task, two in the build)."""
    out = []
    for p in (os.path.dirname(os.path.dirname(os.path.abspath(_N.__file__))),
              os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))):
        if p not in out:
            out.append(p)
    return out


def audit_in_child(folder, max_file_bytes, cpu_s, wall_s, as_mb=AUDIT_AS_MB, python=None):
    """(result, None) of audit(folder) run in a child interpreter (python -I -B -S -c CHILD_BOOT) under prlimit (address
    space as_mb, cpu_s of CPU, no core) and choom -n 1000 (the memory killer's first choice), at most wall_s of wall
    time; (None, why) when the child does not finish with a result (a limit reached, killed, no result line): the
    submission's static checks then fail, closed. The grader's own process reads no byte of the sources. A child that
    cannot be started at all (no prlimit, no interpreter) is the task's failure (RuntimeError)."""
    prl = shutil.which("prlimit") or ("/usr/bin/prlimit" if os.path.isfile("/usr/bin/prlimit") else None)
    if not prl:
        raise RuntimeError("prlimit not found: the static checks cannot run under their limits")
    roots = _package_roots()
    argv = [python or sys.executable, "-I", "-B", "-S", "-c", CHILD_BOOT, str(len(roots))] + roots + [
        "--audit", os.path.abspath(folder), "--max-file-bytes", str(int(max_file_bytes))]
    choom = shutil.which("choom") or ("/usr/bin/choom" if os.path.isfile("/usr/bin/choom") else None)
    if choom:
        argv = [choom, "-n", "1000", "--"] + argv
    argv = [prl, "--as=%d" % (int(as_mb) << 20), "--cpu=%d" % int(cpu_s), "--core=0", "--"] + argv
    env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "LANG": "C.UTF-8"}
    try:
        p = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env,
                           timeout=float(wall_s))
    except subprocess.TimeoutExpired:
        return None, "the static checks of the sources did not finish within %d s" % int(wall_s)
    except OSError as e:
        raise RuntimeError("the static checks' child could not start: %s" % e)
    if p.returncode != 0:
        return None, "the static checks of the sources ended without a result (exit %d)" % p.returncode
    try:
        res = json.loads(p.stdout.decode("utf-8").strip().splitlines()[-1])
    except (ValueError, IndexError, UnicodeDecodeError):
        return None, "the static checks of the sources wrote no result"
    if not isinstance(res, dict) or res.get("format") != "grip-audit/1":
        return None, "the static checks of the sources wrote no result"
    return res, None


def tree_shape(root, entries_max, depth_max, bytes_max):
    """A walk that only lists and lstat's (nothing is opened or read): {"ok", "why", "entries", "bytes", "depth"} of the
    folder `root`, stopped at the first cap passed: more than entries_max entries (files, folders, links, others), an
    entry more than depth_max levels below root, regular files of more than bytes_max bytes in all (apparent size), an
    entry that cannot be listed or inspected. `root` must be a real folder (not a link)."""
    out = {"ok": False, "why": None, "entries": 0, "bytes": 0, "depth": 0}
    try:
        st = os.lstat(root)
    except FileNotFoundError:
        out["why"] = "missing"
        return out
    except OSError:
        out["why"] = "cannot be inspected"
        return out
    if stat.S_ISLNK(st.st_mode):
        out["why"] = "is a symbolic link"
        return out
    if not stat.S_ISDIR(st.st_mode):
        out["why"] = "is not a folder"
        return out
    todo = [(root, 0)]
    while todo:
        d, depth = todo.pop()
        try:
            with os.scandir(d) as it:
                for e in it:
                    out["entries"] += 1
                    if out["entries"] > entries_max:
                        out["why"] = "holds more than %d files and folders" % entries_max
                        return out
                    if depth + 1 > depth_max:
                        out["why"] = "holds an entry more than %d folder levels deep" % depth_max
                        return out
                    try:
                        st = e.stat(follow_symlinks=False)
                    except OSError:
                        out["why"] = "holds an entry that cannot be inspected"
                        return out
                    if stat.S_ISDIR(st.st_mode):
                        out["depth"] = max(out["depth"], depth + 1)
                        todo.append((e.path, depth + 1))
                    elif stat.S_ISREG(st.st_mode):
                        out["bytes"] += st.st_size
                        out["depth"] = max(out["depth"], depth + 1)
                        if out["bytes"] > bytes_max:
                            out["why"] = "is larger than %d MB" % (bytes_max >> 20)
                            return out
        except OSError:
            out["why"] = "holds a folder that cannot be listed"
            return out
    out["ok"] = True
    return out


# ------------------------------------------------------------------------------------------------ namespace-wide counters
_SNMP_KEYS = {("Tcp", "ActiveOpens"): "tcp_active_opens", ("Udp", "OutDatagrams"): "udp_out", ("UdpLite", "OutDatagrams"): "udplite_out",
              ("Icmp", "OutEchos"): "icmp_echo_out"}
_SNMP6_KEYS = {"Udp6OutDatagrams": "udp6_out", "UdpLite6OutDatagrams": "udplite6_out", "Icmp6OutEchos": "icmp6_echo_out"}


def net_counters(snmp="/proc/net/snmp", snmp6="/proc/net/snmp6"):
    """{counter: int} of this network namespace's outbound activity (TCP active opens count v4 and v6)."""
    out = {v: 0 for v in list(_SNMP_KEYS.values()) + list(_SNMP6_KEYS.values())}
    try:
        with open(snmp) as fh:
            lines = [l.split() for l in fh if l.strip()]
        for a, b in zip(lines[0::2], lines[1::2]):
            proto = a[0].rstrip(":")
            for name, val in zip(a[1:], b[1:]):
                k = _SNMP_KEYS.get((proto, name))
                if k:
                    out[k] = int(val)
    except (OSError, ValueError, IndexError):
        out["unreadable"] = 1
    try:
        with open(snmp6) as fh:
            for l in fh:
                f = l.split()
                if len(f) == 2 and f[0] in _SNMP6_KEYS:
                    out[_SNMP6_KEYS[f[0]]] = int(f[1])
    except (OSError, ValueError):
        pass
    return out


class NetCounters(object):
    """`with NetCounters() as nc: <run the program>` -> nc.delta {counter: increase}, nc.total (sum of the increases)."""

    def __init__(self, reader=net_counters):
        self.reader = reader
        self.before = self.after = None
        self.delta = {}
        self.total = 0

    def __enter__(self):
        self.before = self.reader()
        return self

    def __exit__(self, *exc):
        self.after = self.reader()
        self.delta = {k: int(self.after.get(k, 0)) - int(self.before.get(k, 0)) for k in self.before if k != "unreadable"}
        self.total = sum(max(0, v) for v in self.delta.values())
        return False


def _parse_proc_net(path):
    out = {}
    try:
        with open(path) as fh:
            next(fh, None)
            for line in fh:
                f = line.split()
                if len(f) >= 10:
                    out[f[9]] = f[2]
    except OSError:
        pass
    return out


def _is_loopback_or_unspecified(rem):
    ip, _, port = rem.partition(":")
    if port == "0000" or set(ip) <= {"0"}:
        return True
    if len(ip) == 8:
        return ip[-2:].upper() == "7F"
    if len(ip) == 32:
        return ip.upper() == "00000000000000000000000001000000" or (ip[:24].upper() == "0000000000000000FFFF0000" and ip[-2:].upper() == "7F")
    return False


def count_remote_sockets(pids):
    """Remote (non-loopback) inet sockets held by `pids` (the method of `behavior/wire.count_remote_sockets`)."""
    inodes = set()
    for pid in pids:
        fd_dir = "/proc/{}/fd".format(pid)
        try:
            names = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in names:
            try:
                target = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if target.startswith("socket:[") and target.endswith("]"):
                inodes.add(target[8:-1])
    if not inodes:
        return 0
    n = 0
    for name in ("tcp", "tcp6", "udp", "udp6"):
        table = _parse_proc_net("/proc/net/" + name)
        for ino in inodes:
            rem = table.get(ino)
            if rem is not None and not _is_loopback_or_unspecified(rem):
                n += 1
    return n


def uid_pids(uid):
    out = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            if os.stat("/proc/" + d).st_uid == uid:
                out.append(int(d))
        except OSError:
            continue
    return out


# the launch chain's own helpers run as the runner account for a few milliseconds, at the inherited oom_score_adj,
# before `choom` writes the floor and execs the interpreter: told apart by their executable name so they are never
# killed (the interpreter the account finally runs has already had the floor written by `choom`).
LAUNCH_CHAIN_COMMS = frozenset(("runuser", "prlimit", "choom"))


def _proc_comm(pid):
    try:
        with open("/proc/%d/comm" % pid) as fh:
            return fh.read().strip()
    except OSError:
        return ""


class SocketSampler(object):
    """Samples the remote inet sockets of every process of `uid` (or of the `pids_fn()` list) every `period_s`. With
    `oom_floor`, each pass first KILLS (SIGKILL) any of these processes whose `oom_score_adj` reads below that floor and
    counts the kills in `oom_killed`: every process of the program is started at the floor and a legitimate program
    never lowers it, so one found below it has lowered its own value and is stopped (it then ends as a crash under the
    unit's rules). The launch chain's own helpers (runuser / prlimit / choom), which briefly run as the account at the
    inherited value before choom writes the floor, are spared by their executable name."""

    def __init__(self, uid=None, pids_fn=None, period_s=0.5, oom_floor=None):
        self.uid, self.pids_fn, self.period_s = uid, pids_fn, float(period_s)
        self.oom_floor = None if oom_floor is None else int(oom_floor)
        self.max_seen = 0
        self.samples = 0
        self.oom_killed = 0
        self._stop = threading.Event()
        self._t = None

    def _pids(self):
        if self.pids_fn is not None:
            return list(self.pids_fn())
        return uid_pids(self.uid) if self.uid is not None else []

    def _enforce_oom(self, pids):
        for pid in pids:
            try:
                with open("/proc/%d/oom_score_adj" % pid) as fh:
                    v = int(fh.read().strip())
            except (OSError, ValueError):
                continue
            if v >= self.oom_floor:
                continue
            if _proc_comm(pid) in LAUNCH_CHAIN_COMMS:
                continue                                # the launch chain before choom's write: spared
            try:
                os.kill(pid, signal.SIGKILL)
                self.oom_killed += 1
            except OSError:
                continue

    def _run(self):
        while not self._stop.is_set():
            try:
                pids = self._pids()
            except Exception:  # noqa: BLE001
                pids = []
            if self.oom_floor is not None:
                self._enforce_oom(pids)
            try:
                n = count_remote_sockets(pids)
            except Exception:  # noqa: BLE001
                n = 0
            self.samples += 1
            self.max_seen = max(self.max_seen, n)
            self._stop.wait(self.period_s)

    def __enter__(self):
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._t is not None:
            self._t.join(timeout=5.0)
        return False


def _main(argv):
    """The static checks' child (CHILD_BOOT): --audit FOLDER --max-file-bytes N; one JSON line."""
    if len(argv) != 4 or argv[0] != "--audit" or argv[2] != "--max-file-bytes":
        print("usage: --audit FOLDER --max-file-bytes N", file=sys.stderr)
        return 2
    print(json.dumps(audit(argv[1], int(argv[3]))))
    return 0
