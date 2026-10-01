#!/usr/bin/env python3
"""Headless check for JdeRobot/RoboticsInfrastructure#797.

Drops a 1.5 kg box (drone stand-in) onto the car roof in a Visual Lander world
and checks whether it rests on the car and is carried along, or falls through.
"""
import argparse
import math
import re
import subprocess
import sys
import time

BOX = """
		<model name="test_box">
			<pose>{x} {y} 2.6 0 0 0</pose>
			<link name="link">
				<inertial>
					<mass>1.5</mass>
					<inertia><ixx>0.03625</ixx><iyy>0.03625</iyy><izz>0.0625</izz>
					<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
				</inertial>
				<collision name="collision">
					<geometry><box><size>0.5 0.5 0.2</size></box></geometry>
				</collision>
			</link>
		</model>
"""

POSE_RE = re.compile(r"- Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]")


def model_xyz(name):
    out = subprocess.run(["gz", "model", "-m", name, "-p"],
                         capture_output=True, text=True, timeout=30).stdout
    m = POSE_RE.search(out)
    return [float(v) for v in m.group(1).split()] if m else None


def sim_time():
    try:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", "/stats"],
                             capture_output=True, text=True, timeout=20).stdout
    except subprocess.TimeoutExpired:
        return None
    m = re.search(r"sim_time\s*\{\s*sec:\s*(\d+)(?:\s*nsec:\s*(\d+))?", out)
    return int(m.group(1)) + int(m.group(2) or 0) * 1e-9 if m else None


def wait_sim_time(target, wall_timeout=300):
    start = time.time()
    while time.time() - start < wall_timeout:
        t = sim_time()
        if t is not None and t >= target:
            return t
        time.sleep(0.5)
    raise RuntimeError(f"sim time never reached {target}s")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--world", required=True)
    p.add_argument("--label", required=True)
    p.add_argument("--car", nargs=3, type=float, required=True,
                   metavar=("X", "Y", "YAW"), help="car start pose")
    p.add_argument("--expect", choices=["carried", "fell"], required=True)
    a = p.parse_args()

    world = open(a.world, encoding="utf-8").read()
    # No sensors in these worlds without the drone; skip rendering entirely
    world = re.sub(r'<plugin filename="gz-sim-sensors-system".*?</plugin>', "",
                   world, flags=re.S)
    # Roof collision centre is at (0, 0.45) in the car link frame
    x, y, yaw = a.car
    rx = x - 0.45 * math.sin(yaw)
    ry = y + 0.45 * math.cos(yaw)
    world = world.replace("</world>", BOX.format(x=rx, y=ry) + "\t</world>")
    test_world = f"/tmp/{a.label}.world"
    open(test_world, "w", encoding="utf-8").write(world)

    log = open(f"/tmp/{a.label}.log", "w")
    server = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "3", test_world],
                              stdout=log, stderr=subprocess.STDOUT)
    try:
        t1 = wait_sim_time(5)
        box1, car1 = model_xyz("test_box"), model_xyz("car")
        t2 = wait_sim_time(25)
        box2, car2 = model_xyz("test_box"), model_xyz("car")
    finally:
        server.terminate()
        try:
            server.wait(timeout=20)
        except subprocess.TimeoutExpired:
            server.kill()
        log.close()

    print(f"[{a.label}] t={t1:.1f}s box={box1} car={car1}")
    print(f"[{a.label}] t={t2:.1f}s box={box2} car={car2}")

    logtext = open(f"/tmp/{a.label}.log").read()
    problems = [l for l in logtext.splitlines()
                if "WaypointFollower" in l or "Failed to load system plugin" in l]
    for l in problems:
        print(f"[{a.label}] LOG: {l}")

    failures = []
    if a.expect == "fell":
        if box2 is None or box2[2] > 1.0:
            failures.append(f"box should have fallen through, z={box2 and box2[2]}")
    else:
        if problems:
            failures.append("plugin errors in server log")
        if None in (box1, box2, car1, car2):
            failures.append("could not read poses")
        else:
            car_move = math.dist(car1[:2], car2[:2])
            box_move = math.dist(box1[:2], box2[:2])
            print(f"[{a.label}] car moved {car_move:.2f} m, box moved "
                  f"{box_move:.2f} m, box z {box1[2]:.2f} -> {box2[2]:.2f}")
            if car_move < 5.0:
                failures.append(f"car barely moved ({car_move:.2f} m)")
            if min(box1[2], box2[2]) < 1.9:
                failures.append("box is not on the roof")
            if abs(box_move - car_move) > 0.5:
                failures.append("box was not carried along with the car")

    if failures:
        print(f"[{a.label}] FAIL: " + "; ".join(failures))
        sys.exit(1)
    print(f"[{a.label}] PASS ({a.expect})")


if __name__ == "__main__":
    main()
