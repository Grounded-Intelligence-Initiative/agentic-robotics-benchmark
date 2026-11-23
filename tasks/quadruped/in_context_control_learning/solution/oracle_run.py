"""Engine-staged author feasibility script. A participant may not run a script."""
from client import Client
from reference import Controller


if __name__ == "__main__":
    Controller(Client().request).solve()
    print("Author reference completed commissioning and all six stops using ordinary robot calls.", flush=True)
