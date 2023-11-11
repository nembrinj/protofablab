import streamlit as st
import streamlit_authenticator as stauth
import io
import os
from dotenv import load_dotenv
import database as db
import time
import qrcode
from datetime import datetime, timedelta

######ENVIRONMENT_VARIABLES##############################################

load_dotenv(".env")
SALT = os.getenv("SALT")
ADMIN = os.getenv("ADMIN")

st.set_page_config(page_title="UniLock", page_icon=":lock")

######FUNCTIONS##########################################################

def qr_button_state():
    st.session_state.qr_button = True

def qr_button_state_load():
    st.session_state.qr_button_load = True

def load_qr(qr_value):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(qr_value)  # Use the random string as data
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    return img

def get_remaining_seconds(qr_time):
    current_time = datetime.now()
    time_difference = current_time - qr_time
    remaining_seconds = (timedelta(days=0, seconds=time_difference.seconds, microseconds=time_difference.microseconds)).total_seconds()
    return int(remaining_seconds)

######AUTHENTICATION#####################################################

hashed_passwords, usernames, names = db.get_users()

credentials = {"usernames":{}}
        
for u, n, p in zip(usernames, names, hashed_passwords):
    user_dict = {"name": n, "password": p}
    credentials["usernames"].update({u: user_dict})

authenticator = stauth.Authenticate(credentials, "unilock_cookie", SALT, cookie_expiry_days=1)

name, authentication_status, username = authenticator.login("UniLock Login", "main")

######APP################################################################

if authentication_status:

    authenticator.logout("Logout")

    # Admin board
    if username == ADMIN:

        st.subheader(f'Welcome {name}')
        st.header("UniLock - Admin Board")

        tab1, tab2, tab3 = st.tabs(["Dashboard", "Create User", "Manage Users"])

        with tab1:

            st.image("data:image/jpeg;base64,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", width=200)
                        
        with tab2:
            
            with st.form("create_user_form", clear_on_submit = True):

                new_name = st.text_input("Name")
                new_username = st.text_input("Username")
                new_password = st.text_input("Password", type="password")
                submitted = st.form_submit_button("Create User")
                user_created_placeholder = st.empty()

                if submitted:

                    if not new_name or not new_username or not new_password:

                        user_created_placeholder.warning("Please fill in all fields.", icon="⚠️")
                        time.sleep(2)
                        user_created_placeholder.empty()

                    elif new_username in usernames:

                        user_created_placeholder.warning("Please chose another username.", icon="⚠️")
                        time.sleep(2)
                        user_created_placeholder.empty()

                    else:

                        hashed_password = stauth.Hasher([new_password]).generate()[0]
                        db.insert_user(new_username, new_name, hashed_password)
                        user_created_placeholder.success(f"User '{new_name}' with username '{new_username}' successfully created.", icon="✅")
                        hashed_passwords, usernames, names = db.get_users()
                        time.sleep(2)
                        user_created_placeholder.empty()

        with tab3:

            option = st.selectbox(
                'Select ordering...',
                label_visibility='collapsed',
                options=['Names', 'Usernames'],
                index=None,
                placeholder="Select ordering...",
            )
            
            sorted_usernames, sorted_names = usernames, names
            
            if option =='Names':
                data = list(zip(usernames, names))
                sorted_data = sorted(data, key=lambda x: x[0])
                sorted_usernames, sorted_names = zip(*sorted_data)

            elif option =='Usernames':
                data = list(zip(usernames, names))
                sorted_data = sorted(data, key=lambda x: x[1])
                sorted_usernames, sorted_names = zip(*sorted_data)

            for n, u in zip(sorted_names, sorted_usernames):

                with st.expander(f'**{n}** - {u}'):

                    with st.form(key=f'update_user_form_{u}', clear_on_submit = True):

                        new_password = st.text_input("New Password", type="password")
                        submitted = st.form_submit_button("Update Password")
                        password_modified_placeholder = st.empty()

                        if submitted:

                            if not new_password:

                                password_modified_placeholder.warning("Password cannot be empty", icon="⚠️")
                                time.sleep(2)
                                password_modified_placeholder.empty()

                            else:

                                hashed_password = stauth.Hasher([new_password]).generate()[0]
                                db.update_user(u, {'password': hashed_password})
                                password_modified_placeholder.success(f"Password successfully updated.", icon="✅")
                                time.sleep(2)
                                password_modified_placeholder.empty()

                    remove_button = st.button(
                        "Del️ete User",
                        type="primary",
                        key=f'remove_{u}',
                        on_click=db.delete_user,
                        args=[u])

    # User board
    else:

        st.subheader(f'Welcome {name}')
        st.header("UniLock - QR Generator")
        
        qr = db.get_latest_qr(username)

        if qr:
            if 'qr_button_load' not in st.session_state:
                st.session_state.qr_button_load = False

            qr_btn = st.button("Load QR",
                disabled=st.session_state.qr_button_load,
                key='btn_load',
                on_click=qr_button_state_load,
            )

            progress_text = "Click to load a one-time valid QR"
            my_bar = st.progress(0, text=progress_text)
            qr_placeholder = st.empty()

            if st.session_state.qr_button_load:
                del st.session_state.qr_button_load
                
                qr_image = load_qr(qr['qr_value'])
                passed_seconds = get_remaining_seconds(qr['time'])

                qr_bytes = io.BytesIO()
                qr_image.save(qr_bytes, format="PNG")
                qr_placeholder.image(qr_bytes.getvalue(), use_column_width=True)

                # Calculate the number of iterations
                desired_duration = 30
                percentual_update = 0.01
                iterations = int(desired_duration / percentual_update)
                current_time = int(passed_seconds / percentual_update)

                # Progress the bar for the desired duration
                for percent_complete in range(current_time, iterations):
                    time.sleep(percentual_update)
                    remaining_time = desired_duration - (percent_complete * percentual_update) + 1
                    my_bar.progress((percent_complete + 1) / iterations, f'The QR code will disappear in {int(remaining_time)} seconds')

                time.sleep(0.2)
                my_bar.empty()
                qr_placeholder.empty()
                st.rerun()

        else:
            if 'qr_button' not in st.session_state:
                st.session_state.qr_button = False

            qr_btn = st.button("Generate QR",
                disabled=st.session_state.qr_button,
                key='btn',
                on_click=qr_button_state,
            )

            progress_text = "Click to generate a one-time valid QR"
            my_bar = st.progress(0, text=progress_text)
            qr_placeholder = st.empty()

            if st.session_state.qr_button:
                
                # TODO: turn on camera, turn on LED
                qr_image = db.generate_qr_code(username)
                passed_seconds = 0

                qr_bytes = io.BytesIO()
                qr_image.save(qr_bytes, format="PNG")
                qr_placeholder.image(qr_bytes.getvalue(), use_column_width=True)

                # Calculate the number of iterations
                desired_duration = 30
                percentual_update = 0.01
                iterations = int(desired_duration / percentual_update)
                current_time = int(passed_seconds / percentual_update)
                del st.session_state.qr_button

                # Progress the bar for the desired duration
                for percent_complete in range(current_time, iterations):
                    time.sleep(percentual_update)
                    remaining_time = desired_duration - (percent_complete * percentual_update) + 1
                    my_bar.progress((percent_complete + 1) / iterations, f'The QR code will disappear in {int(remaining_time)} seconds')
                
                time.sleep(0.2)
                my_bar.empty()
                qr_placeholder.empty()
                st.rerun()

elif authentication_status == False:
    st.error("Username/password is incorrect", icon="🚨")

elif authentication_status == None:
    st.warning("Please enter your username and password", icon="⚠️")