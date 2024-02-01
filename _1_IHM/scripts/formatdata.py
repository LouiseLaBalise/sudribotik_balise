from datetime import datetime



"""Format bytes numbers (in b, kB, MB or GB)
arg : bytes_size:int
Return a tuple : ( size:int , unit:str ) -> (125, "kB")
"""
def formatBytes(bytes_size:int):
    units = ['b', 'kB', 'MB', 'GB'] #units
    size = bytes_size

    for unit in units:
        if size < 1024.0:
            return f"{size:.1f}", unit
        size /= 1024.0

    #If size is too big return GB by default
    return f"{size:.1f}", 'GB'


"""Format seconds into date
arg : seconds:float
Return a tuple : (date:str , hour:str ) -> ("26 juin. 2023", "15:48:07")
"""
def formatSeconds(seconds:float):
    # Define months
    months = ["jan.", "fév.", "mars", "avr.", "mai", "juin", "juill.", "août", "sept", "oct", "nov", "déc."]
    
    #Get date and time
    dt = datetime.fromtimestamp(seconds)
    
    #Format it
    return (str(dt.day).zfill(2)+" "+months[dt.month-1]+" "+str(dt.year),
          str(dt.hour).zfill(2)+":"+str(dt.minute).zfill(2)+":"+str(dt.second).zfill(2))