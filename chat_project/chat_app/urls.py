from django.urls import path
from chat_app import views

urlpatterns = [
    path('', views.index, name='index'),
    path('send_message', views.send_message, name='send_message'),
    
]
