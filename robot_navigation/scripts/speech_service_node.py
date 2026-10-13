#!/usr/bin/env python3
"""Actual, cancellable speech. No shell expansion and no log-only success."""
import os
import shutil
import subprocess
import tempfile
import threading
import time
import rospy
from std_msgs.msg import Bool, Empty
from robot_navigation.srv import Speak, SpeakResponse

class SpeechService:
    def __init__(self):
        self.lock = threading.Lock()
        self.generation = 0
        self.enabled = True
        self.estop = False
        self.engine = shutil.which('espeak-ng') or shutil.which('espeak')
        self.player = shutil.which('paplay')
        self.voice = rospy.get_param('~voice', 'zh')
        self.device = rospy.get_param('~speaker_device', 'default')
        self.timeout = float(rospy.get_param('~playback_timeout_s', 15.0))
        self.directory = tempfile.TemporaryDirectory(prefix='robot-speech-')
        self.cache = {}
        if self.engine and self.player:
            for bed in (1, 3):
                text = '%d床病人请取药' % bed
                path = os.path.join(self.directory.name, '%d.wav' % bed)
                try:
                    subprocess.run([self.engine, '-v', self.voice, '-s', '175', '-w', path, text],
                                   check=True, timeout=5, stdout=subprocess.DEVNULL)
                    self.cache[text] = path
                except (OSError, subprocess.SubprocessError) as exc:
                    rospy.logerr('[speech] cannot prepare phrase: %s', exc)
        else:
            rospy.logerr('[speech] install espeak-ng and pulseaudio-utils; speech will fail until available')
        self.subs = [rospy.Subscriber('/mission/cancel', Empty, self.cancel),
                     rospy.Subscriber('/emergency_stop', Bool, self.on_estop),
                     rospy.Subscriber('/emergency_stop/reset', Bool, self.on_reset),
                     rospy.Subscriber('/mission/actions_enabled', Bool, self.on_enabled)]
        self.server = rospy.Service('/speak', Speak, self.speak)

    def cancel(self, _):
        self.generation += 1
        self.enabled = False

    def on_estop(self, msg):
        if msg.data:
            self.estop = True
            self.cancel(msg)

    def on_reset(self, msg):
        if msg.data:
            self.estop = False

    def on_enabled(self, msg):
        if msg.data:
            self.enabled = True
        else:
            self.cancel(msg)

    def run(self, args, generation, timeout):
        process = subprocess.Popen(args, stdout=subprocess.DEVNULL)
        deadline = time.monotonic() + timeout
        try:
            while process.poll() is None:
                if rospy.is_shutdown() or self.estop or not self.enabled or generation != self.generation:
                    raise RuntimeError('speech cancelled')
                if time.monotonic() >= deadline:
                    raise RuntimeError('speech playback timeout')
                time.sleep(0.02)
            if process.returncode != 0:
                raise RuntimeError('audio process failed: exit %d' % process.returncode)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=0.5)

    def speak(self, request):
        if not self.lock.acquire(False):
            return SpeakResponse(False, 'speech busy')
        try:
            if not request.text or len(request.text) > 256 or not self.enabled or self.estop:
                raise RuntimeError('invalid text or speech disabled')
            if not self.engine or not self.player:
                raise RuntimeError('espeak(-ng)/paplay is missing')
            generation = self.generation
            path = self.cache.get(request.text)
            if path is None:
                path = os.path.join(self.directory.name, 'custom.wav')
                self.run([self.engine, '-v', self.voice, '-s', '175', '-w', path, request.text], generation, 5)
            args = [self.player]
            if self.device != 'default':
                args.append('--device=' + self.device)
            args.append(path)
            rospy.loginfo('[speech] playing: %s', request.text)
            self.run(args, generation, self.timeout)
            if generation != self.generation or not self.enabled or self.estop:
                raise RuntimeError('speech cancelled')
            return SpeakResponse(True, 'speaker playback finished')
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            rospy.logerr('[speech] %s', exc)
            return SpeakResponse(False, str(exc))
        finally:
            self.lock.release()

if __name__ == '__main__':
    rospy.init_node('speech_service_node')
    SpeechService()
    rospy.spin()
