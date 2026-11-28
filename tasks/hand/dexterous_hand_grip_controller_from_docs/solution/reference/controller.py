"""The privileged reference controller: it echoes the grader's reference commands. It only works with the token file
next to it (without the token the grader sends no reference commands and setup fails). stdlib only."""


class Controller(object):
    def __init__(self, workdir):
        self.workdir = workdir

    def setup(self, record, cell, truth=None, truth_reply=None):
        if truth_reply is None:
            raise RuntimeError("the reference controller needs the grader's reference payload (token)")
        return truth_reply

    def step(self, obs):
        return obs.get("truth_cmd")
