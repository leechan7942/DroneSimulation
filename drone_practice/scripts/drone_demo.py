#!/usr/bin/env python3

import os
import signal
import subprocess
import sys

import rospy


DEFAULT_SCRIPTS = [
    "path_follower.py",
    "dodge.py",
    "land.py",
]


class DroneMainLauncher:
    def __init__(self):
        rospy.init_node("drone_main")

        self.scripts_dir = rospy.get_param("~scripts_dir", self.find_scripts_dir())
        self.script_names = self.get_script_names()
        self.processes = []

        rospy.on_shutdown(self.stop_all_scripts)

    def find_scripts_dir(self):
        current_dir = os.path.dirname(os.path.realpath(__file__))

        try:
            import rospkg

            package_path = rospkg.RosPack().get_path("drone_practice")
            return os.path.join(package_path, "scripts")
        except Exception:
            return current_dir

    def get_script_names(self):
        script_names = rospy.get_param("~scripts", DEFAULT_SCRIPTS)

        if isinstance(script_names, str):
            script_names = [name.strip() for name in script_names.split(",")]

        return [name for name in script_names if name]

    def start_script(self, script_name):
        script_path = script_name
        if not os.path.isabs(script_path):
            script_path = os.path.join(self.scripts_dir, script_name)

        if not os.path.exists(script_path):
            rospy.logwarn("하위 스크립트가 없어 실행하지 않음: %s", script_path)
            return

        if os.access(script_path, os.X_OK):
            command = [script_path]
        else:
            command = [sys.executable, script_path]

        process = subprocess.Popen(
            command,
            cwd=self.scripts_dir,
            preexec_fn=os.setsid,
        )
        self.processes.append((script_name, process))
        rospy.loginfo("하위 스크립트 실행: %s", script_name)

    def start_all_scripts(self):
        for script_name in self.script_names:
            self.start_script(script_name)

        if not self.processes:
            rospy.logwarn("실행된 하위 스크립트가 없음")

    def stop_all_scripts(self):
        for script_name, process in self.processes:
            if process.poll() is not None:
                continue

            rospy.loginfo("하위 스크립트 종료 요청: %s", script_name)
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except OSError:
                process.terminate()

        for script_name, process in self.processes:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                rospy.logwarn("하위 스크립트 강제 종료: %s", script_name)
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except OSError:
                    process.kill()

    def run(self):
        self.start_all_scripts()

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            alive_processes = []
            for script_name, process in self.processes:
                return_code = process.poll()
                if return_code is None:
                    alive_processes.append((script_name, process))
                    continue

                rospy.logwarn(
                    "하위 스크립트 종료 감지: %s, 종료 코드: %s",
                    script_name,
                    return_code,
                )

            self.processes = alive_processes
            rate.sleep()


if __name__ == "__main__":
    try:
        DroneMainLauncher().run()
    except rospy.ROSInterruptException:
        pass
