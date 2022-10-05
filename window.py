from tkinter import *
from PIL import Image
from PIL import ImageTk
import threading
import cv2

def pre_image(pose): #안내 이미지 전처리
    pose = cv2.resize(pose,(150,250))
    pose = cv2.cvtColor(pose,cv2.COLOR_BGR2RGB)
    pose = Image.fromarray(pose)
    return pose


if __name__=="__main__":
    pose1 = cv2.imread("d:/pose1.png")
    pose1 = cv2.resize(pose1,(150,250))
    ##cv2.imshow("pose1",pose1)

    root = Tk()

    pre_pose1 = pre_image(pose1)
    pre_pose1 = ImageTk.PhotoImage(pre_pose1)
    pose1_label = Label(root,image = pre_pose1)
    pose1_label.place(x=10,y=10)
    
    root.mainloop()
    
    a = int(input("ddd"))
    if a ==1:  #hand
        pose1 = cv2.circle(pose1,(80,105),10,(255,100,100),8)
    pose1_point = pre_image(pose1)
    pose1_point = ImageTk.PhotoImage(pose1_point)
    pose1_label.config(image = pose1_point)
##    root = Tk()
##    root.title("자세교정 웹캠")
##    root.geometry("500x300")
##
##    #버튼 추가
##    
##    start_button = Button(root,text = "시작하기",width = 8,height=2)
##    start_button.place(x=400,y=10)
##
##    stay_button = Button(root,text = "멈추기",width = 8,height=2)
##    stay_button.place(x=400,y=60)
##
##    stay_button = Button(root,text = "다시시작",width = 8,height=2)
##    stay_button.place(x=400,y=110)
##
##    end_button = Button(root,text = "끝내기",width = 8,height=2)
##    end_button.place(x=400,y=160)
##
##    pose1 = PhotoImage(file = "d:/pose1.png").subsample(2)
##    pose1_label = Label(root,image = pose1)
##    pose1_label.place(x=10,y=10)
##
##    pose2 = PhotoImage(file = "d:/pose2.png").subsample(2)
##    pose2_label = Label(root,image = pose2)
##    pose2_label.place(x=200,y=10)
##    
##
##    root.mainloop()


    
##
##
##        win1 = Tk()
##        cv2.imshow('front', image_com)
##        image_com = cv2.cvtColor(image_com,cv2.COLOR_BGR2RGB)
##        image_com = Image.fromarray(image_com)
##        image_com = ImageTk.PhotoImage(image_com)
##
##        front_label = Label(win1,image = image_com)
##        front_label.image = image_com
##        front_label.pack()     



##    openfile = Tk()
##    openfile.title("자세교정 웹캠")
##    openfile.geometry("300x350")
##
##    start_button = Button(openfile,text = "시작하기",command = start)
##    start_button.pack()
##
##    start_image = PhotoImage(file = "d:/dog.jpg")
##    front_label = Label(openfile)#image = start_image
##    front_label.pack()
##    openfile.mainloop()
##    

