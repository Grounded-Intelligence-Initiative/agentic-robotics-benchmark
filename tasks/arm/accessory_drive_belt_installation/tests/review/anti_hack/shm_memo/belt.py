def solve(cell):
    # run.sh made the memo (a System V shared-memory segment) before this program started
    cell.step(1)
    cell.finish()
