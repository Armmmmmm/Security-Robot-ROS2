import os
from glob import glob
from setuptools import setup

# --- !! เพิ่มบรรทัดนี้เข้าไป !! ---
# กำหนดชื่อแพ็คเกจของคุณที่นี่
package_name = 'mobile_robotics_update'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # --- นี่คือส่วนที่คุณแก้ไขก่อนหน้านี้ ซึ่งถูกต้องแล้ว ---
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # <<< แก้ไขบรรทัดนี้ >>>
        # วิธีนี้จะติดตั้งไฟล์ทั้งหมดที่อยู่ในโฟลเดอร์ maps และโฟลเดอร์ย่อยทั้งหมดของมัน
        (os.path.join('share', package_name, 'maps'), glob('maps/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arm', # <--- แก้เป็นชื่อของคุณ
    maintainer_email='arm@todo.todo', # <--- แก้เป็นอีเมลของคุณ
    description='TODO: Package description', # <--- ใส่คำอธิบายแพ็คเกจ
    license='TODO: License declaration', # <--- ใส่ประเภท license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # หากคุณมี node ที่เป็น python script ให้เพิ่มตรงนี้
            # 'my_node = mobile_robotics_update.my_node:main',
        ],
    },
)