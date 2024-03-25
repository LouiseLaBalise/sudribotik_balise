from datetime import datetime


def formatBytes(bytes_size:int):
    """
    Format bytes numbers in b, kB, MB or GB.

    Parameters:
        - bytes_size (int): bytes number to format.

    Returns:
        - tuple : size and unit like for exemple (125, "kB").
    """

    units = ['b', 'kB', 'MB', 'GB'] #units
    size = bytes_size

    for unit in units:
        if size < 1024.0:
            return f"{size:.1f}", unit
        size /= 1024.0

    #If size is too big return GB by default
    return f"{size:.1f}", 'GB'



def formatSeconds(seconds:float):
    """
    Format seconds into date.
    
    Parameters:
        - seconds:float
    Returns:
        - tuple : date and hour like for exemple ("26 juin. 2023", "15:48:07").
    """

    # Define months
    months = ["jan.", "fév.", "mars", "avr.", "mai", "juin", "juill.", "août", "sept", "oct", "nov", "déc."]
    
    #Get date and time
    dt = datetime.fromtimestamp(seconds)
    
    #Format it
    return (str(dt.day).zfill(2)+" "+months[dt.month-1]+" "+str(dt.year),
          str(dt.hour).zfill(2)+":"+str(dt.minute).zfill(2)+":"+str(dt.second).zfill(2))