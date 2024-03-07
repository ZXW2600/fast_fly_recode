from fast_fly.planner.time_allocate import AvrTimeAllocater

def test_AvrTimeAllocater():
    waypoints = [ [0, 0, 1], [0, 0, 2], [0, 0, 3], [0, 0, 4], [0, 0, 5]]
    avr_speed = 1
    length_per_sample = 0.1
    loop = False
    init_pos = [0, 0, 0]
    t = AvrTimeAllocater(waypoints, loop, avr_speed, length_per_sample, init_pos)
    print(t.getSamples())
    print(t.getDts())
    print(t.getTimes())

test_AvrTimeAllocater()