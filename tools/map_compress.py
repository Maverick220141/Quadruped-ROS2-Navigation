#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import cv2
import base64
import json

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class MapCompressorNode(Node):
    def __init__(self):
        super().__init__('map_compressor_node')
        
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        self.publisher = self.create_publisher(String, '/map_compressed', 10)
        self.get_logger().info('✅ 地图压缩节点已启动，正在等待小车移动以获取 /map...')

    def map_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            if width == 0 or height == 0:
                self.get_logger().warn("⚠️ 收到长宽为 0 的空地图，跳过处理。")
                return

            # 加了一句 INFO 打印，证明成功进入了回调！
            self.get_logger().info(f"收到地图！分辨率:{resolution}, 尺寸:{width}x{height}，开始压缩...")

            # 1. 将一维 tuple 转换为 numpy 数组
            grid_data = np.array(msg.data, dtype=np.int8)

            # 2. 颜色映射
            img_1d = np.full(grid_data.shape, 127, dtype=np.uint8)
            img_1d[grid_data == 0] = 255
            img_1d[grid_data >= 90] = 0 

            # 3. 重塑为二维图像
            img_2d = img_1d.reshape((height, width))

            # 4. 翻转图像
            img_2d = cv2.flip(img_2d, 0)

            # 5. 压缩为 PNG
            success, buffer = cv2.imencode('.png', img_2d)
            if not success:
                self.get_logger().error("❌ 地图 PNG 压缩失败！")
                return

            # 6. 转为 Base64
            base64_str = base64.b64encode(buffer).decode('utf-8')

            # 7. 构建 JSON
            payload = {
                "image": "data:image/png;base64," + base64_str,
                "info": {
                    "width": width,
                    "height": height,
                    "resolution": resolution,
                    "origin_x": origin_x,
                    "origin_y": origin_y
                }
            }

            # 8. 发布
            out_msg = String()
            out_msg.data = json.dumps(payload)
            self.publisher.publish(out_msg)
            
            # 【关键修改】：将 debug 改为了 info！
            self.get_logger().info('🚀 成功发布压缩地图帧至 /map_compressed !')

        except Exception as e:
            self.get_logger().error(f"❌ 压缩过程发生异常: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MapCompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()