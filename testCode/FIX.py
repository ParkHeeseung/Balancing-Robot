


while True :

    fr = open("pid_K.txt", "r")
    lines = fr.readlines()
    Theta = int(lines[0])
    KP = int(lines[1])
    KI = int(lines[2])
    KD = int(lines[3])

    print("Theta : %d" %Theta)
    print("KP : %d" %KP)
    print("KI : %d" %KI)
    print("KD : %d" %KD)
    print("-------------")
