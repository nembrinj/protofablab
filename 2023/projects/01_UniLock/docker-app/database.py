import os
import random
import qrcode
import string
from dotenv import load_dotenv
from pymongo.mongo_client import MongoClient
from pymongo.server_api import ServerApi
from datetime import datetime, timedelta

######ENVIRONMENT_VARIABLES##############################################

load_dotenv(".env")
MONGO_URI = os.getenv("MONGO_URI")

######MONGODB_CLIENT_INIT################################################

client = MongoClient(MONGO_URI, server_api=ServerApi('1'))
db = client["unilock_db"] # database
users_collection = db["users"] # users collection
qrs_collection = db["qrs"] # qrs collection

######FUNCTIONS##########################################################

def insert_user(username, name, password):
    """Returns the inserted user's _id"""
    user_data = {"username": username, "name": name, "password": password}
    result = users_collection.insert_one(user_data)
    return result.inserted_id

def fetch_all_users():
    """Returns a list of all users"""
    users = users_collection.find()
    return list(users)

def get_user(username):
    """Returns a user document if found, or None"""
    user = users_collection.find_one({"username": username})
    return user

def update_user(username, updates):
    """Updates the user and returns True, or False if not found"""
    result = users_collection.update_one({"username": username}, {"$set": updates})
    return result.modified_count > 0

def delete_user(username):
    """Deletes the user and returns True, or False if not found"""
    result = users_collection.delete_one({"username": username})
    return result.deleted_count > 0

def get_users():
    users = fetch_all_users()
    usernames = [user["username"] for user in users]
    names = [user["name"] for user in users]
    hashed_passwords = [user["password"] for user in users]
    return hashed_passwords, usernames, names

def generate_random_string(length):
    # Generate a random string of the specified length
    characters = string.ascii_letters + string.digits
    return ''.join(random.choice(characters) for _ in range(length))

def generate_qr_code(username):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    random_string = generate_random_string(10)
    insert_qr(username, random_string)
    qr.add_data(random_string)  # Use the random string as data
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    return img

def insert_qr(username, qr_value):
    qr_data = {"username": username, "qr_value": qr_value, "time": datetime.now()}
    result = qrs_collection.insert_one(qr_data)
    return result.inserted_id

def get_latest_qr(username):
    # Define the time threshold for the last 1 minute
    time_threshold = datetime.now() - timedelta(seconds=30)
    
    # Query the database to find the latest QR code for the given username within the time threshold
    query = {"username": username, "time": {"$gte": time_threshold}}
    
    # Sort the results in descending order to get the latest QR code
    latest_qr_cursor = qrs_collection.find(query).sort("time", -1).limit(1)
    
    # Get the count of matching documents
    latest_qr_count = qrs_collection.count_documents(query)
    
    if latest_qr_count > 0:
        return latest_qr_cursor[0]
    else:
        return None