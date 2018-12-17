

while True:
    fr = open("pid_K.txt", "r")
    lines = fr.readlines()
    Theta = int(lines[0])
    KP = int(lines[1])
    KI = int(lines[2])
    KD = int(lines[3])
    fr.close()

    f = open('pid_K.txt', 'w')
    if input() == 1:

        while True:
            if ITheta = input():
                f.write(str(ITheta))
                f.write('\n')
                f.write(str(KP))
                f.write('\n')
                f.write(str(KI))
                f.write('\n')
                f.write(str(KD))
                f.write('\n')
                f.close()
                break

    if input() == 2:
        while True:
            if IKp = input():
                f.write(str(Theta))
                f.write('\n')
                f.write(str(IKp))
                f.write('\n')
                f.write(str(KI))
                f.write('\n')
                f.write(str(KD))
                f.write('\n')
                f.close()
                break

    if input() == 3:
        while True:
            if IKi = input():
                f.write(str(Theta))
                f.write('\n')
                f.write(str(KP))
                f.write('\n')
                f.write(str(Iki))
                f.write('\n')
                f.write(str(KD))
                f.write('\n')
                f.close()
                break


    if input() == 4:
        while True:
            if IKd = input():
                f.write(str(Theta))
                f.write('\n')
                f.write(str(KP))
                f.write('\n')
                f.write(str(KI))
                f.write('\n')
                f.write(str(IKd))
                f.write('\n')
                f.close()
                break
