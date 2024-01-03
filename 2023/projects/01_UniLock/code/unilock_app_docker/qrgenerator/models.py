from django.db import models
from django.contrib.auth.models import User  # Add this import

class QRCode(models.Model):
    user = models.ForeignKey(User, on_delete=models.CASCADE, default=1)
    qr_value = models.TextField(null=True, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return f'QR Code - {self.user} - {self.created_at}'