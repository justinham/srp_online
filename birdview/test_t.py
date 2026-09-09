import requests

# Define the local or server URL
url = "http://127.0.0.1:5000/path_upload_x5_trailer"
# url = "http://127.0.0.1:5000/path_upload_x5_trailer_large_rad"
# url = "http://127.0.0.1:5000/path_upload_x5_trailer_large_rad_front_end"

# Define the array with exactly 5 points
# payload = {"message":"[[0.0, 0.0, 270], [5.0, 0.0, 270], [10.0, 0.0, 270], [10.0, 5.0, 180], [10.0, 7.0, 180]]"}
# payload = {"message":"[[0.0, 0.0, 260], [5.0, 0.0, 260], [14.0, 0.0, 260], [14.0, 10.0, 170], [14.0, 15.0, 170]]"}
# payload = {"message":"[[0.0, 0.0, 270], [5.0, 0.0, 270], [14.0, 0.0, 270], [14.0, 10.0, 180], [14.0, 15.0, 180]]"}
payload = {"message":"[[0.0, 0.0, 270], [5.0, 0.0, 270], [14.0, 4.0, 180], [24.0, 10.0, 180], [24.0, 15.0, 180]]"} # turn
# payload = {"message":"[[0.0, 0.0, 3.8], [0, -4.0, 224], [-7.0, -11.0, 256], [2.4, -14.0, 256], [12.0, -6.0, 256]]"} # turn
# payload = {"message":"[[0.0, 0.0, 31.8], [-2.8, -3.40, 263], [-7.0, -11.0, 256], [-3.1, -8.70, 294], [12.0, -6.0, 256]]"} # turn
# payload = {"message":"[[0.0, 0.0, 0], [0.0, -5.0, 0], [0.0, -10.0, 1], [0, -20.0, 0], [0.5, -25.0, 350]]"}
# payload = {"message":"[[0.0, 0.0, 14.5], [-2.1, -3.20, 177.67], [0.18, -59.30, 111.05], [5.3, -32.8, 111.05], [0.5, -25.0, 350]]"}
payload = {"message": "[[0,0,315.641357421875],[2.3887724876403809,-5.0327811241149902,108.84295654296875],[35.440811157226562,-16.312273025512695,125.92156982421875],[18.694786071777344,-4.1805629730224609,125.92156982421875],[1.948758602142334,7.9511466026306152,125.92156982421875]]"}


## straight
# payload = {"message":"[[0.0, 0.0, 0], [0.0, -5.0, 0], [0.0, -10.0, 1], [0, -20.0, 0], [0.5, -25.0, 350]]"}
## curve right
payload = {"message":"[[0.0, 0.0, 0], [0.0, -5.0, 0], [0.0, -10.0, 1], [10, -35.0, 270], [0.5, -25.0, 270]]"}
## sharp curve 
# payload = {"message":"[[0.0, 0.0, 0], [0.0, -5.0, 0], [0.0, -10.0, 1], [15, -25.0, 240], [0.5, -25.0, 240]]"}

try:
    # Send the POST request using the 'json' parameterc
    # This automatically sets Content-Type to application/json
    response = requests.post(url, json=payload, verify=False)
    
    # Print the response status and JSON message
    print(f"Status Code: {response.status_code}")
    print(f"Response Body: {response.json()}")

except requests.exceptions.RequestException as e:
    print(f"An error occurred: {e}")
