# Coding Agents

> Give a coding agent the maker-sdk skill and the docs, so the scripts it writes use the SDK the way it expects.

- Section: Python SDK

A coding agent writes better scripts for the humanoid robot when it knows how the SDK behaves: which
command does what, what the robot must be doing first, and that guards are the script's job.
Two things give it that: the **maker-sdk skill**, a usage guide the agent loads when you ask
it for robot code, and the docs as plain text in **llms.txt**.

> [!WARNING]
>
> **The agent drives a real robot**
>
> A script an agent writes moves the robot like any other script. Read it before you run it,
> and follow Safety: support the robot, hanging from its gantry hook or seated
> on a stool or bench, and keep the E-Stop in Humanoid Manager open, or be ready to cut power at
> the battery unit.

## Install the Skill

The skill is in the skills/maker-sdk
folder of the SDK repository, in the Agent Skills format, so most coding agents can load it.

### Claude Code

Add the repository as a plugin marketplace, then install the plugin:

```text
/plugin marketplace add maker/maker-sdk
/plugin install maker-sdk@maker
```

`/plugin` then lists `maker-sdk@maker`, and the skill is `maker-sdk:maker-sdk`. To get a
newer version later, run `/plugin update maker-sdk@maker`.

### Codex, pi, oh-my-pi and Other Agents

Install it with the `skills` installer:

```bash
npx skills add maker/maker-sdk
```

It copies the skill to `~/.agents/skills/maker-sdk`, the folder Codex, pi and oh-my-pi read,
and links it for the other agents it finds on your computer. Add `-g -y` to skip the
questions.

### By Hand

Copy the `skills/maker-sdk` folder from the repository into the skills folder your agent
reads: `~/.agents/skills/` for Codex, pi and oh-my-pi, or `~/.claude/skills/` for Claude
Code.

## Try It

Ask your agent for a script, for example:

```text
Using maker-sdk, walk my humanoid robot forward at 0.3 m/s for 3 s and end balancing in place.
```

With the skill, the answer uses the SDK's own calls:

```python
robot.set_velocity(vx=0.3, duration=3.0)  # walks for 3 s, then sends zero velocity
robot.balance()                           # stays in MOVE, balancing in place
```

## What the Skill Teaches

- How to connect in each connection mode and what each one carries.
- The commands, what each waits for, and the errors they raise.
- That the SDK does not refuse a command because of what the robot reports, so a script
  writes its own guard from `robot.get_state()`, as
   does.
- The order to bring the robot up and down: support it before MOVE to STAND and before DAMP.
- Where the full documentation is.

## Give an Agent the Docs

For an agent without the skill, or for questions the skill does not cover, point it at the
docs as plain text:

| File | What it holds |
| - | - |
|  | An index of every docs page, one line each, with a short glossary; the SDK pages are under "Python SDK" |
| sdk/llms-full.txt | The full text of every SDK page in one file |
|  | The full text of every docs page in one file |

Each page is also available as Markdown: add `.md` to its URL, for example
sdk/quickstart.md. Other sections with more
than one page have their own `llms-full.txt` under the section's URL, and llms.txt links to
each one.

## Related

- Quickstart: the first walk, step by step
- Safety: what to check before a script moves the robot, and writing a guard
- Examples: the scripts the skill points to
