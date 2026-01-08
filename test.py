import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import LandedState
import gz_camera_to_frame
import threading
import cv2
import random
import numpy as np

# --- كلاس إدارة واجهة العرض (HUD) ---
class HUDManager:
    def __init__(self):
        self.display_text = "READY TO ANALYZE AUDIO"
        self.result_text = ""
        self.intensity = 0.0
        self.color = (255, 255, 255)
        self.show_details = False

manager = HUDManager()

# --- دالة عرض الكاميرا مع النصوص ---
async def show_camera_stream():
    async for frame in gz_camera_to_frame.get_frames():
        h, w, _ = frame.shape
        
        # خلفية HUD شفافة
        overlay = frame.copy()
        cv2.rectangle(overlay, (20, h - 180), (480, h - 20), (40, 40, 40), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)

        # النص الأساسي (الحالة الحالية)
        cv2.putText(frame, manager.display_text, (40, h - 140), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # عرض تفاصيل النتيجة عند تفعيلها
        if manager.show_details:
            cv2.putText(frame, f"INTENSITY: {manager.intensity:.1f}%", (40, h - 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, manager.color, 2)
            cv2.putText(frame, manager.result_text, (40, h - 65), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # شريط القوة البصري
            bar_w = int((manager.intensity / 110) * 200)
            cv2.rectangle(frame, (40, h - 45), (40 + bar_w, h - 33), manager.color, -1)

        # إطار أصفر وميض أثناء مرحلة التحليل فقط
        if "ANALYZING" in manager.display_text:
            cv2.rectangle(frame, (0, 0), (w, h), (0, 255, 255), 5)

        cv2.imshow("Drone HUD View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# --- دالة إنشاء نقطة المهمة ---
async def create_item(lat,lon,yaw_deg):
    return MissionItem(
        lat, lon, 40, 10, False, 
        float('nan'), float('nan'), MissionItem.CameraAction.NONE, 
        10, float('nan'),3,yaw_deg, float('nan'), 
        MissionItem.VehicleAction.NONE
    )
async def pos_htarget(drone: System, target_min: float, target_max: float):
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= target_min and pos.relative_altitude_m <=  target_max:
            print("Drone reached default takeoff altitude!")
            break

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Drone connected")
    
    # تشغيل الكاميرا في Thread منفصل
    threading.Thread(target=asyncio.run, args=(show_camera_stream(),), daemon=True).start()
  
    async for health in drone.telemetry.health():
     if health.is_home_position_ok:
        print("Home position is OK")
        break
    home = await anext(drone.telemetry.home())
    lat, lon = home.latitude_deg, home.longitude_deg

   
    mission_items = []
    mission_items.append(await create_item(lat +50* 1e-5 , lon +10 *1e-5,-15))#!!!!#0
    mission_items.append(await create_item(lat +60* 1e-5 , lon +30 *1e-5,-30))#!!!!#1
    mission_items.append(await create_item(lat +35* 1e-5 , lon +120*1e-5,150))#!!!!#2
    mission_items.append(await create_item(lat +30* 1e-5 , lon +140*1e-5,150))#!!!!#3
    mission_items.append(await create_item(lat -100 * 1e-5, lon +170 * 1e-5,100))#!!!!#4
    mission_items.append(await create_item(lat -135* 1e-5, lon +110* 1e-5,150)) 
    mission_items.append(await create_item(lat -130* 1e-5, lon +100* 1e-5,165)) 
    mission_items.append(await create_item(lat -125* 1e-5, lon +7* 1e-5,195)) #!!#
    mission_items.append(await create_item(lat -110* 1e-5, lon -5 * 1e-5,180)) #!!#
    mission_items.append(await create_item(lat -45* 1e-5, lon +20* 1e-5,90)) #!!!!#
    mission_items.append(await create_item(lat -30* 1e-5, lon +30* 1e-5,50)) #!!!!#

    #mission_items.append(await create_item(lat -50* 1e-5, lon + 40* 1e-5,float('nan'))) #180.66+75.69

    mission_items.append(await create_item(lat, lon,float('nan')))

    mission_plan = MissionPlan(mission_items)
    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)

    # 1. مرحلة الإقلاع (العنوان ثابت)
    manager.display_text = "READY TO ANALYZE AUDIO"
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(40)
    await drone.action.takeoff()
    await pos_htarget(drone, 39.5, 41.5) 
# 2. بدء الميشن
    print("\n[SYSTEM] Starting mission sequence...")
    await drone.mission.start_mission()

    last_item = -1 

    async for progress in drone.mission.mission_progress():
        if progress.current != last_item:
            
            # أ. استثناء العودة للهوم
            if progress.current == progress.total:
                manager.show_details = False
                manager.display_text = "MISSION ACCOMPLISHED - RETURNING HOME"
                print(f"\n[MISSION UPDATE] Reached point {progress.current}/{progress.total}")
                print("[STATUS] Mission accomplished! Returning to launch point...")
                break 

            # ب. مرحلة التوجه
            manager.show_details = False
            
            
            if progress.current == progress.total - 1:
                manager.display_text = "GO TO HOME LOCATION"
            else:
                manager.display_text = f"GO TO MINARET {progress.current}"
            
            print(f"\n[MISSION UPDATE] Heading to Minaret {progress.current}...")
            print(f"[NAV] Flying to coordinates of Minaret {progress.current}...")

            # --- الانتظار حتى يتوقف الدرون ---
            arrived = False
            while not arrived:
                async for velocity in drone.telemetry.velocity_ned():
                    speed = (velocity.north_m_s**2 + velocity.east_m_s**2 + velocity.down_m_s**2)**0.5
                    if speed < 0.3: 
                        arrived = True
                    break 
                if not arrived:
                    await asyncio.sleep(1.5)#0.5

            # ج. مرحلة التحليل
            print(f"[STATUS] Arrived at Minaret {progress.current}. Stabilizing for analysis...")#تيرمينال 
            manager.display_text = "ANALYZING AUDIO..."#ros
            print(f"[ACTION] Running Audio Frequency Analysis...")#تيرمينال
            
            await asyncio.sleep(6) 

            # د. عرض النتيجة النهائية
            lvl = random.uniform(10, 110)
            manager.intensity = lvl
            manager.show_details = True
            
            if lvl < 80:
                manager.color = (0, 165, 255); status = "LOW LEVEL"
            elif lvl < 91:
                manager.color = (0, 255, 0); status = "NORMAL / STABLE"
            else:
                manager.color = (0, 0, 255); status = "DANGER: HIGH LEVEL"
            
            manager.display_text = "ANALYSIS COMPLETE"
            manager.result_text = f"Audio in Minaret {progress.current}: {status}"
             
             # تحتاج صيانة إذا كانت الحالة (LOW) أو (DANGER)
            if status == "NORMAL / STABLE":
                maintenance_needed = "NO (System is stable)"
                log_icon = "✔"
            else:
                maintenance_needed = "YES (Requires technical check)"
                log_icon = "✘"
            
            # --- تتبع الحالة في التيرمينال ---
            print("\n" + "="*45)
            print(f"| {log_icon} REPORT: MINARET {progress.current}")
            print(f"| Result: {status}")
            print(f"| Intensity Level: {lvl:.2f}")
            print(f"| Maintenance Needed: {maintenance_needed}")
            print("="*45 + "\n")
            await asyncio.sleep(4) 
            last_item = progress.current
           
    # 3. مرحلة العودة والهبوط (استخدام LandedState)
    manager.show_details = False
    manager.display_text = "MISSION ACCOMPLISHED"
    print("Returning to Launch...")
    await drone.action.return_to_launch()

    # استخدام المكتبة المستدعاة للتأكد من ملامسة الأرض
    async for landed in drone.telemetry.landed_state():
        if landed == LandedState.ON_GROUND:
            print("Drone landed safely!")
            manager.display_text = "DRONE LANDED - SYSTEM OFF"
            break # إنهاء البرنامج تماماً بعد الهبوط

if __name__ == "__main__":
    asyncio.run(run())