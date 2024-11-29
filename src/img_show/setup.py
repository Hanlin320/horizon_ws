from setuptools import setup

package_name = 'img_show'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.8/site-packages', ['scripts/web_server.py']),  # 安装 Python 脚本
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'rclpy'],
    zip_safe=True,
)
