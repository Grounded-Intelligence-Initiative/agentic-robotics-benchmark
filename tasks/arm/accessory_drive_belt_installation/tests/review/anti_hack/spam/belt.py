"""probe: command spam: 5000 rounds of get_state + command + tare, a step every 5th."""


def solve(cell):
    cell.set_control("right", "cartesian", 1.0)
    st = cell.get_state()
    p, q = st["right"]["tcp_pos"], st["right"]["tcp_quat"]
    for i in range(5000):
        cell.get_state()
        cell.command(right={"tcp_pos": p, "tcp_quat": q, "width": 0.04 if i % 2 else 0.05})
        cell.tare("left")
        if i % 5 == 0:
            cell.step(1)
