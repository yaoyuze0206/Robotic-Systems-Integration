#!/usr/bin/env python3
"""
QR 扫描实时记录 -> PDF 证明文档
每收到一次 /qr_result 就写一行，比赛结束（Ctrl-C）自动生成 PDF
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import os, csv
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import cm

class QRLogger(Node):
    def __init__(self):
        super().__init__('qr_logger')
        self.sub = self.create_subscription(String, '/qr_result', self.qr_cb, 10)
        log_dir = os.path.expanduser('~/robot_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(
            log_dir, datetime.now().strftime('qr_log_%Y%m%d_%H%M%S.csv'))
        self.f = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['Timestamp', 'QR Content'])
        self.get_logger().info(f'QR 记录保存到 {self.csv_path}')

    def qr_cb(self, msg: String):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.writer.writerow([now, msg.data])
        self.f.flush()
        self.get_logger().info(f'[{now}]  {msg.data}')

    def destroy_node(self):
        self.f.close()
        self.csv2pdf()
        super().destroy_node()

    def csv2pdf(self):
        pdf_path = self.csv_path.replace('.csv', '.pdf')
        c = canvas.Canvas(pdf_path, pagesize=A4)
        c.setFont("Helvetica", 12)
        c.drawString(2*cm, 28*cm, "DASE7503 Robot QR Scan Log")
        c.drawString(2*cm, 27*cm, f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        y = 25*cm
        with open(self.csv_path, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader)  # 跳过表头
            for row in reader:
                y -= 0.6*cm
                if y < 2*cm:          # 换页
                    c.showPage()
                    y = 28*cm
                c.drawString(2*cm, y, f"{row[0]}   {row[1]}")
        c.save()
        self.get_logger().info(f'PDF 证明已生成: {pdf_path}')

def main():
    rclpy.init()
    node = QRLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()