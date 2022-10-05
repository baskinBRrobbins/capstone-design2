print("준비 중...")

#####아두아노 통신--------
import serial as s
import time

arduino = s.Serial('COM4',9600)##이걸로 통신한다.
#---------------------
import cv2
import mediapipe as mp
import numpy as np
import time
import math
import sys
import os

from tkinter import *
from PIL import Image
from PIL import ImageTk
import threading

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
cap_com = cv2.VideoCapture(0) ##0
cap_web = cv2.VideoCapture(1) ##1
global sleeps
sleeps=1

T_F_angrs=[1,1,1,1,1,1]
pre_pose1 = cv2.imread("d:/pose1.png")
pre_pose2 = cv2.imread("d:/pose2.png")

def pre_image(pose): #안내 이미지 전처리
    pose = cv2.resize(pose,(150,250))
    pose = cv2.cvtColor(pose,cv2.COLOR_BGR2RGB)
    pose = Image.fromarray(pose)
    return pose

#각도를 계산해줌
def calculate_angle(a,b,c):
    a = np.array(a) # First
    b = np.array(b) # Mid
    c = np.array(c) # End
    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    #출력범위 [-pi,pi]
    angle = np.abs(radians*180.0/np.pi)
    
    if angle >180.0:
        angle = 360-angle
        
    return angle

def detect_poes_angle(angrs): #각도별 확인 바른자세면 0, 아니면 1
    #목의 좌우각도
    if (angrs[0] >= 80 and angrs[0] <= 110): #완료
        result = [0]
    else:
        result = [1]
    #측면_ 등의 각도
    if (angrs[1] >= 95 and angrs[1] <= 125): #완료
        result.append(0) 
    else:
        result.append(1)
    #측면_ knee와 hip의 높이
    if (angrs[2] <= 0): # 완료
        result.append(1) 
    else:
        result.append(0)
    #측면_목의 각도
    if (angrs[3] >= 140 and angrs[3] <= 170):#완료
        result.append(0) 
    else:
        result.append(1)
    #측면 오른 손목과 코 거리,  왼 손목과 코 거리
    if (angrs[4] <=0.1 or angrs[5] <=0.13): # 완료
        result.append(1) 
    else:
        result.append(0)
   
    #어깨 높이
    if (angrs[6] >= 0.1): #완료
        result.append(1) 
    else:
        result.append(0)
    
  
    return result

def poes_output(angrs):
    if (angrs[1] == 1 or angrs[5] == 1): #측면 등의 각도와 정면 어깨높이의 값들이 범위외일 경우
        if(angrs[1] == 0 and angrs[5] == 1):
            return "1" #어깨를 바르게하세요
        else:
            return "2" #등을 곧게 펴세요
    elif (angrs[0] == 1 or angrs[3] ==1): #목의 정면,목의 측면 각도/ 또한 이경우 등은 바른자세이다
        return "3" #목을 곧게 펴세요 
    elif (angrs[2] == 1): # 목과 등은 바르나 다리자세가 이상한경우
        return "4" #다리를 바르게 하세요
    elif (angrs[4] == 1): # 목,등,다리는 바르나 손위치가 문제인 경우
        return "5" #턱을 괴지 마세요
    else:
        return "0" #바른자세인경우
        
def pose_process(frame_com,frame_cap,pose):
        # Recolor image to RGB
        image_com = cv2.cvtColor(frame_com, cv2.COLOR_BGR2RGB)
        image_com.flags.writeable = False
        # 포즈 탐지
        results_com = pose.process(image_com)
         # Recolor back to BGR
        image_com.flags.writeable = True
        image_com = cv2.cvtColor(image_com, cv2.COLOR_RGB2BGR)
    
        ##        
        image_cap = cv2.cvtColor(frame_cap, cv2.COLOR_BGR2RGB)
        image_cap.flags.writeable = False
        results_web = pose.process(image_cap)

        image_cap.flags.writeable = True
        image_cap = cv2.cvtColor(image_cap, cv2.COLOR_RGB2BGR)

        return image_com,results_com,image_cap,results_web

def side_landmark(land_web):
    #측면 카메라--------------
    wshoulder_right = [land_web[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,land_web[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]#12
    whip_right = [land_web[mp_pose.PoseLandmark.RIGHT_HIP.value].x,land_web[mp_pose.PoseLandmark.RIGHT_HIP.value].y] #24
    wknee_right = [land_web[mp_pose.PoseLandmark.RIGHT_KNEE.value].x,land_web[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]#26
    wwrist_right = [land_web[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,land_web[mp_pose.PoseLandmark.RIGHT_WRIST.value].y] #16
    wwrist_left = [land_web[mp_pose.PoseLandmark.LEFT_WRIST.value].x,land_web[mp_pose.PoseLandmark.LEFT_WRIST.value].y] #15
    whip_left = [land_web[mp_pose.PoseLandmark.LEFT_HIP.value].x,land_web[mp_pose.PoseLandmark.LEFT_HIP.value].y] #23
    wknee_left = [land_web[mp_pose.PoseLandmark.LEFT_KNEE.value].x,land_web[mp_pose.PoseLandmark.LEFT_KNEE.value].y]#25
            
    #0번을 얼굴코드로,오른쪽 측면이므로 (우선) 12번과 24번이용
    wnose = [land_web[mp_pose.PoseLandmark.NOSE.value].x,land_web[mp_pose.PoseLandmark.NOSE.value].y]#w0

    return wshoulder_right,whip_right,wknee_right,wwrist_right,wwrist_left,whip_left,wknee_left,wnose

def front_landmark(land_com):
    #정면 카메라--------------
    #오른쪽
    cshoulder_right = [land_com[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,land_com[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]#12
    celbow_right = [land_com[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,land_com[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y] #14
    cindex_right = [land_com[mp_pose.PoseLandmark.RIGHT_INDEX.value].x,land_com[mp_pose.PoseLandmark.RIGHT_INDEX.value].y] #20
    #왼쪽
    cshoulder_left = [land_com[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,land_com[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y] #11
    celbow_left = [land_com[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,land_com[mp_pose.PoseLandmark.LEFT_ELBOW.value].y] #13
    cindex_left = [land_com[mp_pose.PoseLandmark.LEFT_INDEX.value].x,land_com[mp_pose.PoseLandmark.LEFT_INDEX.value].y] #17
    cshoulder_mid = [(land_com[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x+land_com[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x)/2,(land_com[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y+land_com[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y)/2] #11.5
    cnose = [land_com[mp_pose.PoseLandmark.NOSE.value].x,land_com[mp_pose.PoseLandmark.NOSE.value].y] #0

    return cshoulder_right,celbow_right,cindex_right,cshoulder_left,celbow_left,cindex_left,cshoulder_mid,cnose
    
def side_angr_list(wshoulder_right,whip_right,wknee_right,wwrist_right,wwrist_left,whip_left,wknee_left,wnose):
    #측면 카메라를 이용할 자세각도
    angr1 = calculate_angle(wshoulder_right, whip_right, wknee_right)#12-24-26
    angr1_out = "side_back_angle: " + str(angr1)
    #print(angr1_out)
    #cv2.putText(image_cap, str(angr1_out), (15,15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 0)) #중료하지 않음
            
    angr2 =  wknee_right[1] - whip_right[1]#24-26 hip과 무릎높이차 오른쪽
    angr2_out = "side_leg_height_right: " + str(angr2)
    #print(angr2_out)
    #cv2.putText(image_cap, str(angr2_out), (15,30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 0)) #중료하지 않음

            
    angr3 = calculate_angle(wnose,wshoulder_right, whip_right)#0-12-24
    angr3_out = "side_neck_angle: " + str(angr3)
    #print(angr3_out)
    #cv2.putText(image_cap, str(angr3_out), (15,45), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 0)) 

            
    dis4 = math.sqrt((wnose[0] - wwrist_right[0])*(wnose[0] - wwrist_right[0])+(wnose[1] - wwrist_right[1])*(wnose[1] - wwrist_right[1]))#w0-16
    dis4_out = "right_dis: " + str(dis4)
    #print(dis4_out)
    #cv2.putText(image_cap, str(dis4_out), (15,60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0)) #오른 손목과 코의 거리
            
            
    dis5 = math.sqrt((wnose[0] - wwrist_left[0])*(wnose[0] - wwrist_left[0])+(wnose[1] - wwrist_left[1])*(wnose[1] - wwrist_left[1]))#w0-15
    dis5_out = "left_dis: " + str(dis5)
    #print(dis5_out)
    #cv2.putText(image_cap, str(dis5_out), (15,75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255)) #왼 손목과 코의 거리
    return angr1,angr2,angr3,dis4,dis5

def front_angr_list(cshoulder_right,celbow_right,cindex_right,cshoulder_left,celbow_left,cindex_left,cshoulder_mid,cnose):    
    angr0 = calculate_angle(cnose,cshoulder_mid,cshoulder_right)#0-11.5-12
    angr0_out = "neck_angle: " + str(angr0)
    #print(angr0_out)
    #cv2.putText(image_com, str(angr0_out), (15,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255)) #목의 각도
            
                       
    dis6 = abs(cshoulder_left[1] - cshoulder_right[1])#11-12
    angr6_out = "shoulder_distance: " + str(dis6)
    #print(angr6_out)
    #cv2.putText(image_com, str(angr6_out), (15,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255)) #양 어깨의 높이차이
    return angr0,dis6
def to_arduino(T_F_angrs):
    print("아두아노로 데이터 전송")
    data = poes_output(T_F_angrs) #아두이노로 보낼 데이터 1글자 문자열 보내기
    print("data :",data)
    data = data.encode('utf-8')#qke을땐 decode 보낼땐 encode
    arduino.write(data)
    ##time.sleep(0.3)
    print("\n\n\n")

def front_display(image_com,cnose,cshoulder_mid,cshoulder_right,cshoulder_left):
    ##display를 위한 변수 조정
    he, we = image_com.shape[:2] ##get front

    cnose[0] = int(cnose[0]*we)
    cshoulder_mid[0] = int(cshoulder_mid[0]*we)
    cshoulder_right[0] = int(cshoulder_right[0]*we)
    cshoulder_left[0]= int(cshoulder_left[0]*we)

    cnose[1] = int(cnose[1]*he)
    cshoulder_mid[1] = int(cshoulder_mid[1]*he)
    cshoulder_right[1] = int(cshoulder_right[1]*he)
    cshoulder_left[1]= int(cshoulder_left[1]*he)

    image_com = cv2.line(image_com, (cnose[0], cnose[1]), (cshoulder_mid[0], cshoulder_mid[1]), (0,255,255), 4)
    image_com = cv2.line(image_com, (cshoulder_mid[0], cshoulder_mid[1]), (cshoulder_right[0], cshoulder_right[1]), (0,255,255), 4)
    image_com = cv2.line(image_com, (cshoulder_right[0], cshoulder_right[1]), (cshoulder_left[0], cshoulder_left[1]), (0,0,255), 4)

    return image_com
            
def side_display(image_cap,wshoulder_right,wwrist_right,whip_right,wnose,wknee_right):
    h, w = image_cap.shape[:2] ##get side

    wshoulder_right[1] = int(wshoulder_right[1]*h)
    wwrist_right[1] = int(wwrist_right[1]*h)
    whip_right[1] = int(whip_right[1]*h)
    wnose[1] = int(wnose[1]*h)
    wknee_right[1] = int(wknee_right[1]*h)

    wshoulder_right[0] = int(wshoulder_right[0]*w)
    wwrist_right[0] = int(wwrist_right[0]*w)
    whip_right[0] = int(whip_right[0]*w)
    wnose[0] = int(wnose[0]*w)
    wknee_right[0] = int(wknee_right[0]*w)

    
    image_cap = cv2.line(image_cap, (wshoulder_right[0], wshoulder_right[1]), (wnose[0], wnose[1]), (0,0,255), 4)
    image_cap = cv2.line(image_cap, (wshoulder_right[0], wshoulder_right[1]), (whip_right[0], whip_right[1]), (0,0,255), 4)
    image_cap = cv2.line(image_cap, (whip_right[0], whip_right[1]), (wknee_right[0], wknee_right[1]), (0,0,255), 4)
    image_cap = cv2.circle(image_cap, (wwrist_right[0], wwrist_right[1]), 5, (0,0,255), -1)
    image_cap = cv2.circle(image_cap, (wwrist_left[0], wwrist_left[1]), 5, (0,255,255), -1)
    return image_cap
#결과 프린트
def res_print(T_F_angrs):
    print("정면 목: ",T_F_angrs[0])
    print("측면 등 구부리기: ",T_F_angrs[1])
    print("측면 다리꼬기: ",T_F_angrs[2])
    print("측면 목 굽기: ",T_F_angrs[3])
    print("측면 턱괴기: ",T_F_angrs[4])
    print("정면 어깨 높이: ",T_F_angrs[5])

def camera_site(image_com,image_cap):
    rec1 = cv2.rectangle(image_com,(300,10),(500,250),(255, 255, 0),3)
    cv2.putText(image_com, " head", (250,15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0))

    rec2 = cv2.rectangle(image_cap,(300,70),(450,350),(0, 255, 0),3)
    cv2.putText(image_cap, "shoulder", (250,70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0))
    cv2.putText(image_cap, "hip", (250,350), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0))

    return image_com,rec1,image_cap,rec2
#데몬 쓰레드_ 메인 프로그램  
def main_program():
    ##with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    with mp_pose.Pose(static_image_mode=True,min_detection_confidence=0.5,min_tracking_confidence=0.9) as pose:
        while (cap_com.isOpened() or cap_web.isOpened()) : #영상을 똑바로 받아오면

            if (sleeps == 0):
                continue

        
            ret1, frame_com = cap_com.read()
            ret, frame_cap = cap_web.read()
            image_com,results_com,image_cap,results_web = pose_process(frame_com,frame_cap,pose)
        
            # Extract landmarks
            try:
                #노트북 카메라의 랜드마크
                land_com = results_com.pose_landmarks.landmark ##
                #웹캠의 카메라 랜드마크
                land_web = results_web.pose_landmarks.landmark ##
           

                # Get 각점의 위치를 받아온다
                #측면 카메라--------------
                wshoulder_right,whip_right,wknee_right,wwrist_right,wwrist_left,whip_left,wknee_left,wnose = side_landmark(land_web)
                #정면 카메라--------------
                cshoulder_right,celbow_right,cindex_right,cshoulder_left,celbow_left,cindex_left,cshoulder_mid,cnose = front_landmark(land_com)
           
            
                # Calculate angle 각도측정--------------------
                #측면 카메라를 이용할 자세각도
                angr1,angr2,angr3,dis4,dis5 = side_angr_list(wshoulder_right,whip_right,wknee_right,wwrist_right,wwrist_left,whip_left,wknee_left,wnose)
                #정면 카메라를 이용할 자세각도
                angr0,dis6 = front_angr_list(cshoulder_right,celbow_right,cindex_right,cshoulder_left,celbow_left,cindex_left,cshoulder_mid,cnose)

                #각도가 범위내에 있는지 판별
                angrs = [angr0,angr1,angr2,angr3,dis4,dis5,dis6] 
                global T_F_angrs 
                T_F_angrs = detect_poes_angle(angrs) #판별하는 함수호출
                
                #결과 프린트
                res_print(T_F_angrs)

                #오류코드를 아두이노로---------------------- 
                to_arduino(T_F_angrs)

                ####display를 위한 변수 조정
                ##front
                image_com = front_display(image_com,cnose,cshoulder_mid,cshoulder_right,cshoulder_left)
                ##side
                image_cap = side_display(image_cap,wshoulder_right,wwrist_right,whip_right,wnose,wknee_right)                                   
            except:
                pass
            
       
            #카메라위치 지정
            image_com,rec1,image_cap,rec2 = camera_site(image_com,image_cap)

            cv2.imshow('front', image_com)
            cv2.imshow('front', rec1)
            cv2.imshow('side', image_cap)
            cv2.imshow('side', rec2)


            if cv2.waitKey(10) & 0xFF == 27:
                break

            
            
            ##time.sleep(0.5)
    
        cap_com.release()
        cap_web.release()
        cv2.destroyAllWindows()

##버튼 
def start():
    main_program.start()
    change_s.start()
def end():
    pid = os.getpid()
    os.kill(pid, 2)
def stay():
    global sleeps
    sleeps=0
def restart():
    global sleeps
    sleeps=1
            
def change():

    while 1:
        if (sleeps == 0):
            continue
        pose1_point = cv2.imread("d:/pose1.png")
        pose2_point = cv2.imread("d:/pose2.png")

        print(T_F_angrs)
        if T_F_angrs[4] ==1:  #hand
            pose1_point = cv2.circle(pose1_point,(80,105),10,(255,100,100),8)
            print("hand")

        if T_F_angrs[3] ==1:  #neck
            pose1_point = cv2.circle(pose1_point,(160,130),10,(255,100,100),8)
            print("neck")
        if T_F_angrs[1] ==1:  #back
            pose1_point = cv2.circle(pose1_point,(195,250),10,(255,100,100),8)
            print("back")
        if T_F_angrs[2] ==1:  #leg
            pose1_point = cv2.circle(pose1_point,(85,330),10,(255,100,100),8)
            print("leg")
        

        pose1 = pre_image(pose1_point)
        pose1 = ImageTk.PhotoImage(pose1)
        pose1_label.config(image = pose1)
        

        if T_F_angrs[0] ==1:  #neck
            pose2_point = cv2.circle(pose2_point,(150,150),10,(255,100,100),8)
        if T_F_angrs[5] ==1:  #shoulder
            pose2_point = cv2.circle(pose2_point,(50,140),10,(255,100,100),8)
            pose2_point = cv2.circle(pose2_point,(250,170),10,(255,100,100),8)
            
        pose2 = pre_image(pose2_point)
        pose2 = ImageTk.PhotoImage(pose2)
        pose2_label.config(image = pose2)
        
        time.sleep(1)
        
    
           
root = Tk()
    
if __name__ == "__main__":
    
    
    main_program = threading.Thread(target = main_program,args=())
    main_program.daemon = True

    change_s = threading.Thread(target = change,args=())
    change_s.daemon = True

    
    #root = Tk()
    root.title("자세교정 웹캠")
    root.geometry("500x300")

    #버튼 추가
    start_button = Button(root,text = "시작하기",command = start,width = 8,height=2)
    start_button.place(x=400,y=10)

    stay_button = Button(root,text = "멈추기",command = stay,width = 8,height=2)
    stay_button.place(x=400,y=60)

    stay_button = Button(root,text = "다시시작",command = restart,width = 8,height=2)
    stay_button.place(x=400,y=110)

    end_button = Button(root,text = "끝내기",command = end,width = 8,height=2)
    end_button.place(x=400,y=160)
    
    pose1 = pre_image(pre_pose1)
    pose1 = ImageTk.PhotoImage(pose1)
    pose1_label = Label(root,image = pose1)
    pose1_label.place(x=10,y=10)
    
    pose2 = pre_image(pre_pose2)
    pose2 = ImageTk.PhotoImage(pose2)
    pose2_label = Label(root,image = pose2)
    pose2_label.place(x=200,y=10)
    
    root.mainloop()

 
    
    

    
    

