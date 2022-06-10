import time



class TimeUtils:

    @staticmethod
    def formatted_time(time_stamp):

        struct_now = time.localtime(time_stamp)
        time.strftime("%Y-%m-%d %H:%M:%S %Z", struct_now)
        mlsec = repr(time_stamp).split('.')[1][:2]

        return time.strftime("%M:%S.{}".format(mlsec), struct_now)
