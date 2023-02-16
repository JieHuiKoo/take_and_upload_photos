#! /home/jiehui/anaconda3/envs/tensorflow/bin/python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
import numpy as np
import os

import mysql.connector
from mysql.connector import Error

from take_and_upload_photos.srv import TakeAndUploadPhoto,TakeAndUploadPhotoResponse
from datetime import datetime


from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


print("Python Version: " + str(sys.version_info[0]) + '.' + str(sys.version_info[1]))
print("OpenCV Version: " + str(cv2.__version__))

def get_location_index(cursor, db_values, connection):

    sql = "SELECT location_id FROM locations WHERE location_name='" + db_values["locations"]["location_name"] + "'"
    cursor.execute(sql)
    row = cursor.fetchone()
    
    if (row is None):
        sql = "INSERT INTO locations (location_name, location_address) VALUES (%s, %s)"
        val = (db_values["locations"]["location_name"], db_values["locations"]["location_address"])
        cursor.execute(sql, val)
        connection.commit()

        sql = "SELECT LAST_INSERT_ID();"
        cursor.execute(sql)
        row = cursor.fetchone()

    return row[0]

def get_action_index(cursor, db_values, connection):

    sql = "INSERT INTO actions (action_name, action_start_datetime, action_success) VALUES (%s, %s, %s)"
    val = (db_values["actions"]["action_name"], db_values["actions"]["action_start_datetime"], db_values["actions"]["action_success"])
    cursor.execute(sql, val)
    connection.commit()

    sql = "SELECT LAST_INSERT_ID();"
    cursor.execute(sql)
    row = cursor.fetchone()

    return row[0]

def insert_photo(cursor, db_values, connection, location_idx, action_idx):

    sql = "INSERT INTO photos_taken (photo_file_location, photo_datetime, location_id, action_id) VALUES (%s, %s, %s, %s)"
    val = (db_values["photos_taken"]["photo_file_location"], db_values["photos_taken"]["photo_datetime"], location_idx, action_idx)
    print(val)
    cursor.execute(sql, val)
    connection.commit()

    sql = "SELECT LAST_INSERT_ID();"
    cursor.execute(sql)
    row = cursor.fetchone()

    return row[0]

def insert_to_db(db_values):
    try:
        connection = mysql.connector.connect(host='localhost',
                                            database='doggobot',
                                            user='root',
                                            password='')
        if connection.is_connected():
            cursor = connection.cursor(buffered=True)

            location_idx = get_location_index(cursor, db_values, connection)
            action_idx = get_action_index(cursor, db_values, connection)
            insert_photo(cursor, db_values, connection, location_idx, action_idx)

    except Error as e:
        print("Error while connecting to MySQL", e)
    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()

def imgmsg_to_cv2(img_msg):
    rgb8_flag = 0
    if img_msg.encoding != "bgr8":
        rgb8_flag = 1
        # rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    if rgb8_flag:
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)

    return image_opencv

def cv2_to_imgmsg(cv_image, encoding):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def generate_file_name(now):
    return int(now.strftime('%Y%m%d%H%M%S%f'))

file_num = 0
def save_image(image, now, save_flag):
    
    server_resource_folder_location = "/home/jiehui/webdir/doggobot_picture_vault/resources"
    file_location = "/taken_photos/" + str(generate_file_name(now)) + ".jpg"
    full_file_location = server_resource_folder_location + file_location
    
    if save_flag: 
        print("Saving Image: " + full_file_location)
        cv2.imwrite(full_file_location, image)
    return file_location

def process_image(msg):
    
    global drawImg 

    # Declare the cvBridge object
    drawImg = imgmsg_to_cv2(msg)

def handle_null_service_call(req):
    if req.action_start_datetime == '':
        req.action_start_datetime, _ = get_current_datetime()
    if req.action_name == '':
        req.action_name = "User Initiated Photo"
    if req.action_success == '':
        req.action_success = 1
    if req.location_name == '':
        req.location_name = "Not Available"
    if req.location_address == '':
        req.location_address = "Not Available"
    return req
    
def get_current_datetime():
    now = datetime.now()
    formatted_time = now.strftime('%Y-%m-%d %H:%M')
    return formatted_time, now


def handle_take_and_upload_photo(req):
    current_datetime, now = get_current_datetime()
    file_location = save_image(drawImg, now, 1)
    req = handle_null_service_call(req)
    db_values = {
        "locations": {"location_name": req.location_name, "location_address": req.location_address},
        "actions": {"action_name":req.action_name, "action_start_datetime":req.action_start_datetime, "action_success":int(req.action_success)},
        "photos_taken": {"photo_file_location":file_location, "photo_datetime":current_datetime}
        }
    print(db_values)
    
    insert_to_db(db_values)

    return TakeAndUploadPhotoResponse(True)

def start_node():
    rospy.init_node('segmented_colour')
    rospy.loginfo('upload_photo service started')
    rospy.Subscriber("/armCamera/color/image_rect_color/", Image, process_image)
    s = rospy.Service('TakeAndUploadPhotos', TakeAndUploadPhoto, handle_take_and_upload_photo)    
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()


    except rospy.ROSInterruptException:
        pass