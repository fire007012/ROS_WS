#!/usr/bin/env python3
"""Standalone speaker test; config selects TTS text or an existing audio file."""
import os
import math
import shutil
import subprocess
import tempfile
import time
import rospy

class SpeechTest:
    def __init__(self):
        self.mode = rospy.get_param('~mode', 'text')
        self.text = str(rospy.get_param('~text', '1床病人请取药'))
        self.audio_file = str(rospy.get_param('~audio_file', ''))
        self.voice = str(rospy.get_param('~voice', 'zh'))
        self.device = str(rospy.get_param('~speaker_device', 'default'))
        self.speed = int(rospy.get_param('~speed', 175))
        self.volume = int(rospy.get_param('~volume', 100))
        self.timeout = float(rospy.get_param('~playback_timeout_s', 15.0))
        if self.mode not in ('text','file') or not math.isfinite(self.timeout) or self.timeout <= 0 or not 80 <= self.speed <= 450 or not 0 <= self.volume <= 200:
            raise ValueError('mode must be text/file and timeout positive')
        self.player = shutil.which('paplay')
        self.engine = shutil.which('espeak-ng') or shutil.which('espeak')
        if not self.player:
            raise RuntimeError('paplay not found; install pulseaudio-utils')
        path = None
        temporary_path = None
        process = None
        try:
            if self.mode == 'file':
                path = os.path.expanduser(self.audio_file)
                if not path or not os.path.isfile(path):
                    raise RuntimeError('audio_file does not exist: '+path)
            else:
                if not self.engine:
                    raise RuntimeError('espeak-ng/espeak not found')
                if not self.text.strip():
                    raise ValueError('text must not be empty')
                with tempfile.NamedTemporaryFile(prefix='speech-test-',suffix='.wav',delete=False) as stream:
                    temporary_path = path = stream.name
                subprocess.run([self.engine,'-v',self.voice,'-s',str(self.speed),'-a',str(self.volume),'-w',path,self.text],check=True,timeout=5)
            args=[self.player]
            if self.device != 'default':args.append('--device='+self.device)
            args.append(path)
            rospy.loginfo('[speech_test] mode=%s file=%s',self.mode,path)
            process=subprocess.Popen(args)
            deadline=time.monotonic()+self.timeout
            while process.poll() is None:
                if rospy.is_shutdown():raise RuntimeError('playback cancelled by ROS shutdown')
                if time.monotonic()>=deadline:raise RuntimeError('playback timeout')
                time.sleep(.02)
            if process.returncode != 0:raise RuntimeError('paplay exit %d'%process.returncode)
            rospy.loginfo('[speech_test] playback finished successfully')
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=.5)
            if temporary_path and os.path.isfile(temporary_path):os.unlink(temporary_path)

if __name__ == '__main__':
    rospy.init_node('speech_test_node')
    try:
        SpeechTest()
    except (ValueError,RuntimeError,OSError,subprocess.SubprocessError) as error:
        rospy.logerr('[speech_test] %s',error)
        raise SystemExit(1)
