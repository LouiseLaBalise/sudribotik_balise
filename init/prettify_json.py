import os

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)


def prettify(json_filepath):
    """
    Prettify a json file to be human readable.

    json_filepath (str)     ->      json file to prettify.

    Return function success.
    """
    #Quit if parameter is not a json file
    if json_filepath.split('.')[-1] != "json":
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : Ce fichier n'est pas un fichier json. Impossible de l'enjoliver.")
        return False

    #Open the json file
    with open(json_filepath, 'r') as file:
            content = file.read()

            quote_or_sq_brace = False
            coma = False
            coma_and_space = False
            result = '' #used to redo a new file with appropiate spacing

            #Loop over each character to see if there is a quote
            for char in content:

                #Handle curly braces
                if char=="{":
                    result+=char+"\n\t"
                    continue
                if char=="}":
                    result+="\n"+char
                    continue

                #Add a line jump after each <,> placed before a quote or a closing square brace
                if quote_or_sq_brace and char == ",":
                    result += ",\n\n\t"
                #Add a line jump and some tabulations before each <[> placed after a coma and space
                elif coma_and_space and char == "[":
                    result += "\t\t\t\t\t ["
                else:
                    result += char


                #If <"> or <]> or <,> or <, > is detected directly look for the next char
                if char == '"' or char == "]":
                    quote_or_sq_brace = True
                elif char == ",":
                    coma = True
                elif coma and char == " ":
                    coma_and_space = True
                else :
                    quote_or_sq_brace = False
                    coma = False
                    coma_and_space = False

    #Save result
    with open(json_filepath, 'w') as file:
        file.write(result)


    return True