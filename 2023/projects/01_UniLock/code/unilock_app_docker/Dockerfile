FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Set working directory
WORKDIR /code

# Copy the current directory contents into the container at /app
COPY . /code

# Install any needed packages specified in requirements.txt
RUN pip install -r requirements.txt

# Set up Django environment variables
ENV DJANGO_SETTINGS_MODULE=unilock_app.settings

# Start Django db (SQLite in this case)
RUN python manage.py makemigrations
RUN python manage.py migrate

# Create Django superuser (admin:admin)
RUN echo "from django.contrib.auth.models import User; User.objects.create_superuser('admin', 'admin@example.com', 'admin')" | python manage.py shell

# Start the Django development server
CMD ["python", "manage.py", "runserver", "0.0.0.0:8000"]
