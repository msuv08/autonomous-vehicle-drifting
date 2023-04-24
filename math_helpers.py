# this function leverages a technique known as interpolation.
def get_value_at_time(time, times, values):

    if time < times[0]:
        return values[0]
    elif time > times[-1]:
        return values[-1]
    left, right = 0, len(times)-1
    while right-left > 1:
        mid = (left+right)//2
        if time > times[mid]:
            left = mid
        elif time < times[mid]:
            right = mid
        else:
            return values[mid]
    return values[left]+(values[right]-values[left])*(time-times[left])/(times[right]-times[left])