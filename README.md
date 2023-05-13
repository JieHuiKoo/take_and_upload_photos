# take_and_upload_photos

About: This package continually subscribes to the images that are published by the cameras. When a valid request is received, the image will be stored locally and an SQL query is generated to insert the metadata into the database as described in doggobot_picture_vault package.

To run:
```
rosrun take_and_upload_photos upload_photo.py
```
Make sure that the doggobot_picture_vault package is downloaded and XAMPP is running.
