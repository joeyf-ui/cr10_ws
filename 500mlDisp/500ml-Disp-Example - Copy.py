import threading
from devonics_api import DevonicsApi
from dobot_api import DobotApi,DobotApiMove,DobotApiDashboard, MyType
from time import sleep
import numpy as np
import schedule

def connectRobot():
    try:
        ip = "192.168.5.1"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004


        print("Connecting...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print("Connection Successful!")
        return dashboard, move, feed
    except Exception as e:
        print("Failed to connect")
        raise e
    
def connectTool(dashboard):
    try:
        ip = "127.0.0.1"
        port = 60000
        slave_id = 1
        rtu = 1
        model = "PGC-50"

        Tool = DevonicsApi(dashboard,ip,port,slave_id,rtu)
        print("Tool Connection Successful!")
        return Tool
    except Exception as e:
        print("Tool Connection Failed")
        raise e

def get_feed(feed: DobotApi):
    global feedBackData
    global current_actual
    global payload
    global centerX
    global centerY
    global centerZ
    hasRead = 0

    data = bytes()
    while hasRead < 1440:
        temp = feed.socket_dobot.recv(1440 - hasRead)
        if len(temp) > 0:
            hasRead += len(temp)
            data += temp

    a = np.frombuffer(data, dtype=MyType)
    if hex((a['test_value'][0])) == '0x123456789abcdef':
        # Refresh Properties
        feedBackData = a
        current_actual = a["tool_vector_actual"][0]
        running_status = a["running_status"][0]
        payload = a["load"][0]
        centerX = a["center_x"][0]
        centerY = a["center_y"][0]
        centerZ = a["center_z"][0]
        # print("Running_Status", running_status)
        # print("Payload:", feedBackData["load"][0])
    sleep(0.001)

def runRobot(dashboard, move, Tool):
    print("move to above tank")
    dashboard.SpeedFactor(40)
    move.ServoJ(-31.0221, -2.8491, -84.2978, -90.8372, -88.9419, -179.22655) 
    sleep(6)

    print("discharge")
    Tool.setSpeed(300)
    move.Sync()
    sleep(1)
    Tool.setPosition(56500)
    move.Sync()
    sleep(18)
    
    print("move into tank")
    dashboard.SpeedFactor(20)
    move.ServoP(506.1660, -540.4417, 220.6578, -88.1298, -0.7438, -122.5535) 
    sleep(10)

    print("suck water")
    CylinderP=int(25000)
    Tool.setPosition(CylinderP)
    move.Sync()
    sleep(23)

    print("move to above tank")
    dashboard.SpeedFactor(40)
    move.ServoP(506.1660, -540.4417, 695.5943, -88.1298, -0.7438, -122.5535) 
    sleep(4)

    perPlantWaitTime = 3
    print("move to Unit 1 plant 8")
    move.ServoP(143.233, -960.976, 400, -88.1356, -0.7449, -166.4269) 
    sleep(2)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setSpeed(200)
    move.Sync()
    sleep(1)
    Tool.setPosition(CylinderP)
    move.Sync()
    sleep(perPlantWaitTime)


    print("move to Unit 1 plant 9")
    move.ServoP(143.233, -863.9020, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 10")
    move.ServoP(143.233, -778.926, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 11")
    move.ServoP(143.233, -690.5601, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 5")
    move.ServoP(67.2810, -733.8015, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 6")
    move.ServoP(67.2810, -825.5445, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 7")
    move.ServoP(67.2810, -914.2372, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 1")
    move.ServoP(-7.9190, -956.0672, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 2")
    move.ServoP(-7.9190, -871.7491, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 3")
    move.ServoP(-7.9190, -782.0326, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 4")
    move.ServoP(-7.9190, -686.8896, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    #unit 2
    print("move to Unit 2 plant 8")
    #x=-352.575
    move.ServoP(143.233-495.808, -960.976, 400, -88.1356, -0.7449, -166.4269) 
    sleep(5)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 9")
    move.ServoP(143.233-495.808, -863.9020, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)


    print("move to Unit 2 plant 10")
    move.ServoP(143.233-495.808, -778.926, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 11")
    move.ServoP(143.233-495.808, -690.5601, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 5")
    move.ServoP(67.2810-495.808, -733.8015, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 6")
    move.ServoP(67.2810-495.808, -825.5445, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 7")
    move.ServoP(67.2810-495.808, -914.2372, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 1")
    move.ServoP(-7.9190-495.808, -956.0672, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 2")
    move.ServoP(-7.9190-495.808, -871.7491, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 3")
    move.ServoP(-7.9190-495.808, -782.0326, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 4")
    move.ServoP(-7.9190-495.808, -686.8896, 400, -88.1356, -0.7449, -166.4269) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+30000//25
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to water tank")
    move.ServoP(506.1660, -540.4417, 695.5943, -88.1298, -0.7438, -122.5535) 
    sleep(4)

def main():
    dashboard, move, feed = connectRobot()
    Tool = connectTool(dashboard)

    feed_thread = threading.Thread(target=get_feed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    dashboard.EnableRobot(1.0,0,0,30)

    schedule.every().day.at("09:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("10:43").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("13:03").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("15:20").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("18:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("21:10").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("02:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("06:00").do(runRobot,dashboard, move, Tool)

    
    while True:
        schedule.run_pending()
        sleep(1)

main()
