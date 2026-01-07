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
async def create_item(lat,lon):
    return MissionItem(
        lat, lon, 15, 10, False, 
        float('nan'), float('nan'), MissionItem.CameraAction.NONE, 
        10, float('nan'),3,float('nan'), float('nan'), 
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

    # الحصول على إحداثيات البيت
    home = await anext(drone.telemetry.home())
    lat, lon = home.latitude_deg, home.longitude_deg

    # --- إضافة الـ 12 نقطة يدوياً كما طلبت ---
    mission_items = []
    mission_items.append(await create_item(lat +45* 1e-5 , lon +10 *1e-5))
    mission_items.append(await create_item(lat +60* 1e-5 , lon +30 *1e-5))
    mission_items.append(await create_item(lat +40* 1e-5 , lon +120*1e-5))
    mission_items.append(await create_item(lat +35* 1e-5 , lon +140*1e-5))
    mission_items.append(await create_item(lat -100 * 1e-5, lon +160 * 1e-5))
    mission_items.append(await create_item(lat - 90 * 1e-5, lon +35 * 1e-5))#كانت 150 و 30
    mission_items.append(await create_item(lat, lon))

    mission_plan = MissionPlan(mission_items)
    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)

    # 1. مرحلة الإقلاع (العنوان ثابت)
    manager.display_text = "READY TO ANALYZE AUDIO"
    await drone.action.arm()
    await drone.action.takeoff()
    #await pos_htarget(drone, 10.5, 11.2) # انتظار حتى يرتفع الدرون ويثبت العنوان

    # 2. بدء الميشن والتسلسل الذكي
    print("Starting mission...")
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        # أ. مرحلة التوجه للمنارة (تختفي النتائج القديمة)
        manager.show_details = False
        manager.display_text = f"GO TO LOCATION {progress.current}"
        await asyncio.sleep(2) 

        # ب. مرحلة التحليل (تختفي جملة الموقع ويطبع ANALYZING فقط)
        manager.display_text = "ANALYZING SIGNALS..."
        await asyncio.sleep(4)

        # ج. مرحلة النتيجة النهائية
        manager.display_text = "ANALYSIS COMPLETE"
        manager.show_details = True
        
        # توليد قراءات عشوائية للألوان والحالة
        lvl = random.uniform(10, 110)
        manager.intensity = lvl
        if lvl < 80:
            manager.color = (0, 165, 255); status = "LOW LEVEL"
        elif lvl < 91:
            manager.color = (0, 255, 0); status = "NORMAL / STABLE"
        else:
            manager.color = (0, 0, 255); status = "DANGER: HIGH LEVEL"
        
        manager.result_text = f"Audio in Minaret Number {progress.current} is {status}"
        
        await asyncio.sleep(4) # تثبيت النتيجة على الشاشة قبل النقطة التالية
        
        if progress.current == progress.total:
            break

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