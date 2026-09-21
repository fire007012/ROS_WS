#!/usr/bin/env python3
import tkinter as tk
import rospy
from std_msgs.msg import String, Float32, Bool
from rospy import AnyMsg
import struct
from robot_navigation.msg import QrResult

class StatusWindow:
    def __init__(self):
        self.fullscreen=rospy.get_param('~fullscreen', True)
        self.root=tk.Tk(); self.root.title('送药巡诊机器人状态'); self.root.configure(bg='#101820')
        if self.fullscreen: self.root.attributes('-fullscreen', True)
        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))
        self.labels={}
        self.make('title','送药巡诊机器人',28,'#4dd0e1')
        for key,text in [('task','当前任务：等待启动'),('scan','识别状态：未开始'),('bed1','1号床条码：---'),('bed3','3号床条码：---'),('hr','心率：--- bpm'),('temp','体温：--- °C'),('measure','测量状态：未开始'),('esp','ESP32：离线')]: self.make(key,text,18,'#f5f5f5')
        rospy.Subscriber('/robot_display',String,lambda m:self.set('task','当前任务：'+m.data.replace('\n',' | ')))
        rospy.Subscriber('/qr_result',QrResult, self.qr)
        rospy.Subscriber('/barcode_bed1',String,lambda m:self.set('bed1','1号床条码：'+m.data))
        rospy.Subscriber('/barcode_bed3',String,lambda m:self.set('bed3','3号床条码：'+m.data))
        rospy.Subscriber('/vitals/heart_rate',Float32,lambda m:self.set('hr','心率：%.1f bpm'%m.data))
        rospy.Subscriber('/vitals/temperature',Float32,lambda m:self.set('temp','体温：%.1f °C'%m.data))
        rospy.Subscriber('/vitals/measuring',Bool,lambda m:self.set('measure','测量状态：'+('测量中' if m.data else '已停止')))
        rospy.Subscriber('/vitals/status',String,lambda m:self.set('esp','ESP32：'+m.data))
        self.root.after(100,self.pump)
    def make(self,key,text,size,color):
        label=tk.Label(self.root,text=text,font=('DejaVu Sans',size,'bold' if key=='title' else 'normal'),fg=color,bg='#101820',anchor='w',padx=30,pady=8); label.pack(fill='x'); self.labels[key]=label
    def set(self,key,text): self.labels[key].config(text=text)
    def qr(self,m): self.set('scan','识别状态：护士台二维码 '+str(m.first_bed)+str(m.first_box)+'，下一站 '+str(m.second_bed)+str(m.second_box))
    def pump(self):
        if not rospy.is_shutdown(): self.root.after(100,self.pump)
    def run(self): self.root.mainloop()
if __name__=='__main__':
    rospy.init_node('robot_status_display_node'); w=StatusWindow(); w.run()



