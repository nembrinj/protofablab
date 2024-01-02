from django.urls import path
from .views import index, generate_qr, turn_on_camera, close_door

urlpatterns = [
    path('', index, name='index'),
    path('turn_on_camera/', turn_on_camera, name='turn_on_camera'),
    path('generate_qr/', generate_qr, name='generate_qr'),
    path('close_door/', close_door, name='close_door'),
]