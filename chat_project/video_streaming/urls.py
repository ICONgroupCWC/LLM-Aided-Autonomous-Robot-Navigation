from django.urls import path
from video_streaming import views


urlpatterns = [
    path('', views.index, name='index'),
    path('video_feed/', views.video_feed, name='video_feed'),  # Make sure the URL pattern matches the one referred to in your template
]