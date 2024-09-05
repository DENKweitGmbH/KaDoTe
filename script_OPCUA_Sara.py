from opcua import Client
from opcua import ua
from opcua.crypto import uacrypto
import time

def findNodesByName(node,name,resu):
    for childId in node.get_children():
        ch = client.get_node(childId)
        nodeName = str(ch.get_browse_name())
        if nodeName[-(len(name) + 1): -1] == name:
            print(nodeName)
            resu.append(ch)
        if ch.get_node_class() == ua.NodeClass.Object:
            findNodesByName(ch, name, resu)
            
def addNewUser(client,username,password):
    obj = client.get_node("ns=2; s=/Methods")
    addUser = client.get_node("ns=2; s=/Methods/AddUser")
    changePassword = client.get_node("ns=2; s=/Methods/ChangeMyPassword")
    obj.call_method(addUser, username)
    obj.call_method(changePassword, username, password, password)

def getUserList(client):
    obj = client.get_node("ns=2; s=/Methods")
    userList = client.get_node("ns=2; s=/Methods/GetUserList")
    return obj.call_method(userList)

def deleteUser(client, username):
    obj = client.get_node("ns=2; s=/Methods")
    delete = client.get_node("ns=2; s=/Methods/DeleteUser")
    obj.call_method(delete, username)

def changePassword(client,oldPassword, newPassword):
    obj = client.get_node("ns=2; s=/Methods")
    change = client.get_node("ns=2; s=/Methods/ChangeMyPassword")
    obj.call_method(change, oldPassword, newPassword, newPassword)

def getAccessRights(client,user=None):
    obj = client.get_node("ns=2; s=/Methods")
    if user == None:
        access = client.get_node("ns=2; s=/Methods/GetMyAccessRights")
        return obj.call_method(access)
    else:
        access = client.get_node("ns=2; s=/Methods/GetUserAccessRights")
        return obj.call_method(access,user)

#### Possible access arguments: ####
#"StateRead"    Zustandsdaten – NC, Kanal, Achse, Lesezugriff
#"StateWrite"   Zustandsdaten – NC, Kanal, Achse, Schreibzugriff
#"FrameRead"    Nullpunktverschiebungen, Lesezugriff
#"FrameWrite"   Nullpunktverschiebungen, Schreibzugriff
#"SeaRead"      Settingdaten, Lesezugriff
#"SeaWrite"     Settingdaten, Schreibzugriff
#"TeaRead"      Maschinendaten, Lesezugriff
#"TeaWrite"     Maschinendaten, Schreibzugriff
#"ToolRead"     Werkzeug- und Magazindaten, Lesezugriff
#"ToolWrite"    Werkzeug- und Magazindaten, Schreibzugriff, Werkzeugverwaltungsmethoden
#"DriveRead"    Antriebsdaten, Lesezugriff
#"DriveWrite"   Antriebsdaten, Schreibzugriff
#"GudRead"      Anwenderdaten, Lesezugriff
#"GudWrite"     Anwenderdaten, Schreibzugriff
#"FsRead"       Dateisystem, Lesezugriff
#"FsWrite"      Dateisystem, Schreibzugriff
#"PlcRead"      PLC, Lesezugriff
#"PlcWrite"     PLC, Schreibzugriff
#"PlcReadSymDBx" PLC, Lesezugriff für symbolische Adresse (x gibt den Namen der symbolischen Adresse an)1
#"PlcWriteSymDBx" PLC, Schreibzugriff für symbolische Adresse (x gibt den Namen der symbolischen Adresse an)1
#"AlarmRead"    Ermöglicht das Abonnieren von Alarmen
#"RandomRead"   Dynamisch, Lesezugriff
#"RandomWrite"  Dynamisch, Schreibzugriff
#"SinuReadAll"  Alle genannten Lesezugriffe
#"SinuWriteAll" Alle genannten Schreibzugriffe
#"ApWrite"      Ermöglicht, die Methode "Select" aufzurufen
#"CsomReadx"    CSOM Lesezugriff (x gibt die Namespace-Nummer an, mögliche Werte: 3-9)2
#"CsomWritex"   CSOM Schreibzugriff (x gibt die Namespace-Nummer an, mögliche Werte: 3-9)
#"ADVRead"      Überwachung von ADV-Ereignissen, Aufrufen von ADV-Anforderungsmethoden
#"ADVWrite"     Zugriff für OPC UA Advanced-Methoden, die das System ändern (Schreibzugriff).

def deleteUserAccess(client, user, access):
    obj = client.get_node("ns=2; s=/Methods")
    delete = client.get_node("ns=2; s=/Methods/DeleteUserAccess")
    obj.call_method(delete,user,access)

def giveUserAccess(client, username, access):
    obj = client.get_node("ns=2; s=/Methods")
    giveAccess = client.get_node("ns=2 ;s=/Methods/GiveUserAccess")
    obj.call_method(giveAccess,username,access)

# take_picture places the robot to the coordinates coord and returns the final positions when coordinates reached
# at the end the robot is ready to teka a picture, for which it can directly associate the corresponding coordinates
def take_picture(client,coord):
    R11 = client.get_node("ns=2; s=/Channel/Parameter/R[11]")
    R12 = client.get_node("ns=2; s=/Channel/Parameter/R[12]")
    R13 = client.get_node("ns=2; s=/Channel/Parameter/R[13]")
    R14 = client.get_node("ns=2; s=/Channel/Parameter/R[14]")
    R15 = client.get_node("ns=2; s=/Channel/Parameter/R[15]")
    R16 = client.get_node("ns=2; s=/Channel/Parameter/R[16]")
    R11.set_value(coord[0])
    R12.set_value(coord[1])
    R13.set_value(coord[2])
    R14.set_value(coord[3])
    R15.set_value(coord[4])
    R16.set_value(coord[5])
    R1 = client.get_node("ns=2; s=/Channel/Parameter/R[1]")
    R0 = client.get_node("ns=2; s=/Channel/Parameter/R[0]")
    R0.set_value(1.0)
    X = client.get_node("ns=2; s=/Channel/GeometricAxis/aaAcsRel[1]")
    Y = client.get_node("ns=2; s=/Channel/GeometricAxis/aaAcsRel[2]")
    Z = client.get_node("ns=2; s=/Channel/GeometricAxis/aaAcsRel[3]")
    while R1.get_value()==0.0:
        print("R1 = "+str(R1.get_value()))
        time.sleep(0.1)
    print("R1 = "+str(R1.get_value()))
    print("Picture is taken")
    R1.set_value(0.0)
    return client.get_values([X,Y,Z])
    

def work_through(client,liste):
    for element in liste:
        if element[0]=="screw":
            R11 = client.get_node("ns=2; s=/Channel/Parameter/R[21]")
            R12 = client.get_node("ns=2; s=/Channel/Parameter/R[22]")
            R13 = client.get_node("ns=2; s=/Channel/Parameter/R[23]")
            R14 = client.get_node("ns=2; s=/Channel/Parameter/R[24]")
            R15 = client.get_node("ns=2; s=/Channel/Parameter/R[25]")
            R16 = client.get_node("ns=2; s=/Channel/Parameter/R[26]")
            R11.set_value(element[1])
            R12.set_value(element[2])
            R13.set_value(element[3])
            R14.set_value(element[4])
            R15.set_value(element[5])
            R16.set_value(element[6])
            R1 = client.get_node("ns=2; s=/Channel/Parameter/R[1]")
            R0 = client.get_node("ns=2; s=/Channel/Parameter/R[0]")
            R0.set_value(2.0)
            while R0.get_value()!=0.0:
                time.sleep(0.1)
            print("screw is removed")
            


# target example: Sinumerik/FileSystem/Sub Program/subprg.spf
def transferSub(client, filename, target, overwrite=True ):
    obj = client.get_node("ns=2; s=/Methods")
    copy2server = client.get_node("ns=2; s=/Methods/CopyFileToServer")
    obj.call_method(copy2server,target, filename, overwrite)

def trans(client):
    t = time.time()
    transferSub(client, b"G1 X0 \n G1 X10 \n M17", "Sinumerik/FileSystem/Sub Program/simple.spf", True)
    i=1
    while True:
        if time.time()-t > 2:
            t = time.time()
            if i == 1:
                transferSub(client, b"G1 X20 \n G1 X10 \n M17", "Sinumerik/FileSystem/Sub Program/simple.spf", True)
                i = 0
            else:
                transferSub(client, b"G1 X0 \n G1 X10 \n M17", "Sinumerik/FileSystem/Sub Program/simple.spf", True)


class SubHandler(object):

    """
    Subscription Handler. To receive events from server for a subscription
    data_change and event methods are called directly from receiving thread.
    Do not do expensive, slow or network operation there. Create another 
    thread if you need to do such a thing
    """

    def datachange_notification(self, node, val, data):
        print("Python: New data change event", node, val)

    def event_notification(self, event):
        print("Python: New event", event)

if __name__ == "__main__":
    client = Client("opc.tcp://134.109.8.245:4840") #OpcUaClient:PASSWORT@134.109.8.238:4840")
    client.set_user("OpcUaClient")
    client.set_password("PASSWORT")
   # client.set_security(ua.SecurityPolicyType.Basic256Sha256_Sign, ua.MessageSecurityMode.SignAndEncrypt)
    client.connect()
    print("Client is connected")


    root = client.get_root_node()
    print("Root is", root)
    print("childs of root are: ", root.get_children())
    print("name of root is", root.get_browse_name())
    R = client.get_node("ns=2; s=/Channel/Parameter/R[1]")
    print("R = "+str(R.get_value()))
    print("data value = "+str(R.get_data_value()))
    print("path = "+str(R.get_path()))
    print("browse name = "+str(R.get_browse_name()))
    print("display name = "+str(R.get_display_name()))
    print("data type = "+str(R.get_data_type()))
    print("data type as variant = "+str(R.get_data_type_as_variant_type()))
    print("access level = "+str(R.get_access_level()))
    print("user access = "+str(R.get_user_access_level()))
    print("node class = "+str(R.get_node_class()))
    print("description = "+str(R.get_description()))
    print("done")





    def speedTest():
        Rparam = []
        nb_values = 101
        for i in range(1,nb_values):
            Rpar = client.get_node("ns=2; s=/Channel/Parameter/R["+str(i)+"]")
            Rparam.append(Rpar)

        t = time.time()
        Rparam_val = client.get_values(Rparam)
        print(Rparam_val)
        tt = time.time()
        dif = tt-t
        print("Duration of get_values with "+str(nb_values)+" values = "+str(dif)+" s")
        t = time.time()
        Rparam_val2 = []
        for node in Rparam:
            Rparam_val2.append(node.get_value())
        tt = time.time()
        dif = tt-t
        print("Duration of get_value loop with "+str(nb_values)+" values = "+str(dif)+" s")


        newRparam = [1.0 for k in range(nb_values)]
        t = time.time()
        client.set_values(Rparam, newRparam)
        tt = time.time()
        dif = tt-t
        print("Duration of set_values with "+str(nb_values)+" values = "+str(dif)+" s")
        t = time.time()
        for i in range(len(Rparam)):
            Rparam[i].set_value(Rparam_val[i])
        tt = time.time()
        dif = tt-t
        print("Duration of set_value loop with "+str(nb_values)+" values = "+str(dif)+" s")
        
        Var = Rparam[1]   
    
        Var.set_value(1.0)
        t1 = time.time()
        Var.get_value()
        tt1 = time.time()
        dif1 = tt1*1000-t1*1000
        print("get_value 1 value duration = "+str(dif1))
        Var.set_value(0.0)
        Var.set_value(1.0)
 #       Var.set_value(0.0)
  #      Var.set_value(3.0)
  #      Var.set_value(0.0)
  #      Var.set_value(4.0)
  #      Var.set_value(0.0)
  #      Var.set_value(5.0)
        t = time.time()
        Var.set_value(0.0)
        tt = time.time()
        dif2 = tt*1000-t*1000
    
        print("set_value 1 value duration = "+str(dif2))

        
  #  myVar = client.get_node("ns=2; s=/NC/_N_NC_GD4_ACX/_DREI")
    
    # subscribing to a variable node
   # handler = SubHandler()
   # sub = client.create_subscription(200, handler)
   # handle = sub.subscribe_data_change(myVar)
   # time.sleep(0.1)

    # List of interesting variable nodes
    # [X,Y,Z]: "ns=2; s=/Channel/GeometricAxis/aaAcsRel[1:3]"   current Machine position
    # À vérifier: F: "ns=2; s=/Channel/GeometricAxis/actFeedRate"
    # À vérifier: speed: "ns=2; s=/Channel/MachineAxis/actSpeedRel"
    # À vérifier: tool bas position: "ns=2; s=/Channel/MachineAxis/actToolBasePos[1:3]"
    # À vérifier: G0 mode: "ns=2; s=/Channel/State/G0Mode"
