from django.contrib import admin

from .models import QRCode

class QRCodeAdmin(admin.ModelAdmin):
  list_display = ("user", "qr_value", "created_at",)

admin.site.register(QRCode, QRCodeAdmin)