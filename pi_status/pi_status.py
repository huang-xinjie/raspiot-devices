import http.server
import json
import socketserver
import subprocess
import threading
import time
import uuid

import qrcode
from PIL import Image
from RPi import GPIO
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306


class PiStatus(object):
    VCC = 7  # VCC为7 GND为9
    GPIO_LOCK = False

    class HttpHandler(http.server.BaseHTTPRequestHandler):
        device = None
        port = 5000

        def do_GET(self):
            if self.path == '/attrs':
                self.send_response(200)
                self.send_header('Content-type', 'text/json')
                self.end_headers()
                attrs = self.device.get_attrs()
                self.wfile.write(json.dumps(attrs).encode())
            else:
                self.send_error(404, 'Resource not found')

        def do_PUT(self):
            if self.path not in ['/attr']:
                self.send_error(404, 'Resource not found')
                return

            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            req_body = json.loads(body)
            print(f'set attr request: {req_body}')

            attr, value = req_body.get('attr'), req_body.get('value')
            self.device.oled_display_setter(attr, value)
            self.send_response(200)
            self.send_header('Content-type', 'text/json')
            self.end_headers()
            attrs = self.device.get_attrs()
            self.wfile.write(json.dumps(attrs).encode())

    def __init__(self):
        self.oled = None
        self.ips = self.get_ips()
        self.always_display = False
        self.ip_onshow = self.ips and self.ips[0] or ''
        self.HttpHandler.device = self

    def __del__(self):
        self.always_display = False
        self.cleanup_gpio(PiStatus.VCC)

    def setup_gpio(self, pin):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(pin, GPIO.OUT)
        # GPIO.output(pin, GPIO.LOW)
        GPIO.output(pin, GPIO.HIGH)

    def cleanup_gpio(self, pin):
        GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()

    def setup_oled(self):
        in_failure = False
        while not self.oled:
            try:
                serial = i2c(port=1, address=0x3C)
                self.oled = ssd1306(serial)  # 这里改ssd1306, ssd1325, ssd1331, sh1106
                print('OLED setup finished.')
            except Exception as e:
                if not in_failure:
                    print(str(e) + ' You can press Ctrl+C to exit or mount the OLED.')
                    in_failure = True
                time.sleep(10)

    @staticmethod
    def get_ips():
        ips = subprocess.check_output(['hostname', '-I'])
        return ips.decode().strip().split()

    @staticmethod
    def get_mac():
        mac = uuid.UUID(int=uuid.getnode()).hex[-12:]
        return ":".join([mac[e:e + 2] for e in range(0, 11, 2)])

    @staticmethod
    def get_temp():
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as t:
            temp = int(t.read()) / 1000
        return temp

    def build_qr_img(self, text, size):
        def make_qrcode(info):
            qr = qrcode.QRCode(version=2, error_correction=qrcode.constants.ERROR_CORRECT_L,  # 二维码的纠错范围
                               box_size=2, border=1)  # 二维码点像素 2;  距图像外围边框距离
            qr.add_data(info)
            qr.make(fit=True)
            return qr.make_image()

        qrImg = make_qrcode(text).convert("RGBA")
        img = Image.new("RGBA", size, "white")
        pos = ((size[0] - qrImg.width) // 2, 0)
        img.paste(qrImg, pos)
        return img

    def show_pi_status(self):
        if PiStatus.GPIO_LOCK:
            print('GPIO locking..')
            return

        PiStatus.GPIO_LOCK = True
        ordinate, qr_code_img = 5, None
        self.setup_gpio(PiStatus.VCC)
        self.setup_oled()

        while True:
            cur_ips = self.get_ips()
            if set(cur_ips) != set(self.ips) or not qr_code_img:
                self.ips, qr_code_img = cur_ips, self.build_qr_img(cur_ips, self.oled.size)

            if not self.oled:
                time.sleep(5)
                continue

            self.oled.show()
            with canvas(self.oled) as draw:
                draw.rectangle(self.oled.bounding_box, outline="white", fill="black")
                draw.text((2, ordinate), f"temp: {self.get_temp()}°C", fill="white")
                draw.text((2, ordinate + 10), f"ip: {self.ips and self.ips[0] or ''}", fill="white")
                for idx in range(1, len(self.ips)):
                    draw.text((2, ordinate + (idx + 1) * 10), f"    {self.ips[idx]}", fill="white")
            time.sleep(10)

            self.oled.display(qr_code_img.convert(self.oled.mode))
            time.sleep(20)

            if not self.always_display:
                self.oled.hide()
                time.sleep(60)
        PiStatus.GPIO_LOCK = False
        print('OLED display stopped. GPIO unlocked.')

    def oled_display_setter(self, attr, value):
        if attr == 'Always display':
            target = bool(str(value).lower() == 'true')
            if target != self.always_display:
                self.always_display = target
        elif attr == 'IP':
            if value in self.ips:
                self.ip_onshow = value
            elif self.ip_onshow not in self.ips:
                self.ip_onshow = self.ips and self.ips[0] or ''

    def get_attrs(self):
        if not (self.ip_onshow and self.ip_onshow in self.ips):
            self.ip_onshow = self.ips and self.ips[0] or ''
        attrs = [{"type": "select",
                  "name": "IP",
                  "value": self.ip_onshow,
                  "value_constraint": {"options": self.ips}},
                 {"type": "text",
                  "name": "Temp",
                  "value": f'{self.get_temp()}°C',
                  "read_only": True},
                 {"type": "switch",
                  "name": "Always display",
                  "value": self.always_display}]
        return attrs

    def run(self):
        threading.Thread(target=self.show_pi_status, args=()).start()
        with socketserver.TCPServer(('', self.HttpHandler.port), self.HttpHandler) as httpd:
            print(f'listening at http://0.0.0.0:{self.HttpHandler.port}')
            httpd.serve_forever()


if __name__ == '__main__':
    device = PiStatus()
    device.run()
