from flask import Flask, render_template, Response, request
import cv2
import datetime, time
import os, sys
import numpy as np
from threading import Thread


global capture,rec_frame, grey, switch, neg, face, rec, out 
capture=0
grey=0
neg=0
face=0
switch=1
rec=0

#make shots directory to save pics
try:
    os.mkdir('./shots')
except OSError as error:
    pass

#Load pretrained face detection model    
# net = cv2.dnn.readNetFromCaffe('./saved_model/deploy.prototxt.txt', './saved_model/res10_300x300_ssd_iter_140000.caffemodel')

#instatiate flask app  
app = Flask(__name__, template_folder='./templates')


camera = cv2.VideoCapture(0)

# Function to detect cracks in the image and calculate magnitude
def detect_cracks(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply a simple threshold to segment cracks
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours of the cracks
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours on a blank image
    # contour_image = np.zeros_like(image)
    contour_image = image
    cv2.drawContours(contour_image, contours, -1, (0, 0, 255), 2)
    
    # Filter contours based on area and draw bounding boxes
    magnitude_factors = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:  # Area greater than 100 pixels (equivalent to 1 square centimeter)
            magnitude_factors.append(area)
            # Get bounding box coordinates and draw the box
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(contour_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    # # Save grayscale image with cracks and their magnitude
    # for i, magnitude in enumerate(magnitude_factors):
    #     if magnitude > 0:
    #         # Save only if the area is greater than 1 square centimeter
    #         cv2.imwrite(f'crack/crack_detected_image_{i+1}.jpg', contour_image)
    
    return gray, contour_image, magnitude_factors

# def record(out):
#     global rec_frame
#     while(rec):
#         time.sleep(0.05)
#         out.write(rec_frame)


def gen_frames():  # generate frame by frame from camera
    global out, capture,rec_frame
    while True:
        success, frame = camera.read() 
        gray_image, frame, magnitude_factors = detect_cracks(frame)
            # cv2.imshow('Original', frame)
            # cv2.imshow('crack', frame_with_contours)
        if success:
            for i, magnitude in enumerate(magnitude_factors):
                if magnitude > 0:
                    capture = 1
            if(face):                
                # frame= detect_face(frame)
                pass
            if(grey):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if(neg):
                frame=cv2.bitwise_not(frame)    
            if(capture):
                capture=0
                now = datetime.datetime.now()
                p = os.path.sep.join(['shots', "crack_{}.png".format(str(now).replace(":",''))])
                cv2.imwrite(p, frame)
            
            if(rec):
                rec_frame=frame
                frame= cv2.putText(cv2.flip(frame,1),"Recording...", (0,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),4)
                frame=cv2.flip(frame,1)
            
                
            try:
                ret, buffer = cv2.imencode('.jpg', cv2.flip(frame,1))
                frame_with_contours = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_with_contours + b'\r\n')
            except Exception as e:
                pass
                
        else:
            pass


@app.route('/')
def index():
    return render_template('index.html')
    
    
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_crack')
def video_feed_crack():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame_with_contours')

@app.route('/requests',methods=['POST','GET'])
def tasks():
    global switch,camera
    if request.method == 'POST':
        if request.form.get('click') == 'Capture':
            global capture
            capture=1
        elif  request.form.get('grey') == 'Grey':
            global grey
            grey=not grey
        elif  request.form.get('neg') == 'Negative':
            global neg
            neg=not neg
        elif  request.form.get('face') == 'Face Only':
            global face
            face=not face 
            if(face):
                time.sleep(4)   
        elif  request.form.get('stop') == 'Stop/Start':
            
            if(switch==1):
                switch=0
                camera.release()
                cv2.destroyAllWindows()
                
            else:
                camera = cv2.VideoCapture(0)
                switch=1
        elif  request.form.get('rec') == 'Start/Stop Recording':
            global rec, out
            rec= not rec
            if(rec):
                now=datetime.datetime.now() 
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                out = cv2.VideoWriter('vid_{}.avi'.format(str(now).replace(":",'')), fourcc, 20.0, (640, 480))
                #Start new thread for recording the video
                thread = Thread(target = record, args=[out,])
                thread.start()
            elif(rec==False):
                out.release()
                          
                 
    elif request.method=='GET':
        return render_template('index.html')
    return render_template('index.html')


if __name__ == '__main__':
    # app.run()
    # app.run(host="0.0.0.0")
    app.run(host="10.42.0.186")
    
camera.release()
cv2.destroyAllWindows()     