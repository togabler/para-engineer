stop_blink = 1
robotType = 'quattro' # Models: 'delta', 'quattro' or '6rus'
import threading

if __name__ == '__main__':
    import startWebsite # imports are here to avoid circular calling of imports
    import startRobot

    monitoring_thread = threading.Thread(target = startRobot.startRobot)
    monitoring_thread.start()

    startWebsite.app.run(debug=False, port=5000, host='0.0.0.0')

