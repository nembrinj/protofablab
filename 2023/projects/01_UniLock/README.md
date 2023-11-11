To instantiate the entire application, you'll need to run `docker-compose` for both the Streamlit app in the `unilock_app` directory and the MongoDB container in the `unilock_db` directory.

Before you begin, ensure that you have the required environment variables defined in an `.env` file located in the `./unilock` directory. These environment variables should include any necessary secrets or configurations for your application.

MONGO_INITDB_ROOT_USERNAME=admin
MONGO_INITDB_ROOT_PASSWORD=admin
MONGO_INITDB_DATABASE=unilock_db
MONGO_URI=mongodb://admin:admin@unilock_mongo:27017/?authMechanism=DEFAULT
SALT=helloworld
ADMIN=admin

1. Open a terminal and navigate to the root directory of your project (`unilock`).

2. First, go to the `unilock_db` directory where your MongoDB container is defined:

   ```bash
   cd unilock_db
   ```

3. Run `docker-compose` to start the MongoDB container, which will automatically create the 'unilock-network' network:

   ```bash
   docker-compose up -d
   ```

   The `-d` flag runs the containers in the background.

4. Once the MongoDB container is up and running, switch back to the root directory of your project (`unilock`):

   ```bash
   cd ..
   ```

5. Go to the `unilock_app` directory where your Streamlit app is located:

   ```bash
   cd unilock_app
   ```

6. Run `docker-compose` to start the Streamlit app container:

   ```bash
   docker-compose up -d
   ```

   The `-d` flag runs the containers in the background.

Now, you should have both the Streamlit app and MongoDB containers running. Your Streamlit app should be accessible at `http://localhost:8501`.

Ensure that you have your environment variables correctly defined in the .env file for your application to work as expected.