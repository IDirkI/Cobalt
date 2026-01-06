import pandas as pd
import matplotlib.pyplot as plt

CSV_FILE = "../../results/kinematics/ik_arm.csv"


def read_robot_csv(path):
    df = pd.read_csv(path)

    header = list(df.columns)
    rows = df.to_numpy(dtype=float).tolist()

    return header, rows


def parse_row(header, row):
    x_goal = row[0]
    y_goal = row[1]

    coords = row[2:]
    links = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]

    return (x_goal, y_goal), links


def plot_robot(header, row):
    (x_goal, y_goal), links = parse_row(header, row)

    xs = [p[0] for p in links]
    ys = [p[1] for p in links]

    ax = plt.gca()
    ax.axhline(links[0][0], color='black', linewidth=0.8)
    ax.axvline(links[0][1], color='black', linewidth=0.8)

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, "-o", label="Robot Links")
    plt.plot(x_goal, y_goal, "b*", markersize=10, label="Goal")

    plt.title("Robot Chain")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    ax.grid(True,alpha=0.15)
    plt.legend()
    plt.show()


def animate_robot(header, rows):
    import time

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))

    for row in rows:
        (x_goal, y_goal), links = parse_row(header, row)
        xs = [p[0] for p in links]
        ys = [p[1] for p in links]

        ax.clear()
        ax.axhline(links[0][0], color='black', linewidth=0.8)
        ax.axvline(links[0][1], color='black', linewidth=0.8)
        ax.plot(xs, ys, "-o",  color='#6ca348')
        plt.plot(x_goal, y_goal, "*", color='#8528bf', markersize=10, label="Goal")
        ax.set_title("Robot Chain")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.axis("equal")
        ax.grid(True,alpha=0.15)

        plt.pause(0.05)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    header, rows = read_robot_csv(CSV_FILE)

    # Plot the last row (final configuration)
    #plot_robot(header, rows[-1])

    # Uncomment to animate:
    animate_robot(header, rows)
