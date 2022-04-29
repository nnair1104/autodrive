import matplotlib.pyplot as plt

mIoU_all = []
mIoU_background_all = []
mIoU_car_all = []
mIoU_junction_all = []
mIoU_road_all = []
l1_all = []

mIoU_all_val = []
mIoU_background_all_val = []
mIoU_car_all_val = []
mIoU_junction_all_val = []
mIoU_road_all_val = []
l1_all_val = []


def plot():
    with open("training_miou_background.txt", "r") as mIoU_b:
        for value in mIoU_b:
            store_mIoU_b = value.split(",")
            store_mIoU_b.pop()
    for i in store_mIoU_b:
        mIoU_background_all.append(float(i))
    with open("evaluation_miou_background.txt", "r") as mIoU_b_v:
        for value in mIoU_b_v:
            store_miou_b_v = value.split(",")
            store_miou_b_v.pop()
    for i in store_miou_b_v:
        mIoU_background_all_val.append(float(i))

    with open("training_miou_car.txt", "r") as mIoU_c:
        for value in mIoU_c:
            store_mIoU_c = value.split(",")
            store_mIoU_c.pop()
    for i in store_mIoU_c:
        mIoU_car_all.append(float(i))
    with open("evaluation_miou_car.txt", "r") as mIoU_c_v:
        for value in mIoU_c_v:
            store_miou_c_v = value.split(",")
            store_miou_c_v.pop()
    for i in store_miou_c_v:
        mIoU_car_all_val.append(float(i))

    with open("training_miou_junction.txt", "r") as mIoU_j:
        for value in mIoU_j:
            store_mIoU_j = value.split(",")
            store_mIoU_j.pop()
    for i in store_mIoU_j:
        mIoU_junction_all.append(float(i))
    with open("evaluation_miou_junction.txt", "r") as mIoU_j_v:
        for value in mIoU_j_v:
            store_miou_j_v = value.split(",")
            store_miou_j_v.pop()
    for i in store_miou_j_v:
        mIoU_junction_all_val.append(float(i))

    with open("training_miou_road.txt", "r") as mIoU_r:
        for value in mIoU_r:
            store_mIoU_r = value.split(",")
            store_mIoU_r.pop()
    for i in store_mIoU_r:
        mIoU_road_all.append(float(i))
    with open("evaluation_miou_road.txt", "r") as mIoU_r_v:
        for value in mIoU_r_v:
            store_miou_r_v = value.split(",")
            store_miou_r_v.pop()
    for i in store_miou_r_v:
        mIoU_road_all_val.append(float(i))

    with open("training_miou.txt", "r") as mIoU:
        for value in mIoU:
            store_mIoU = value.split(",")
            store_mIoU.pop()
    for i in store_mIoU:
        mIoU_all.append(float(i))

    with open("evaluation_miou.txt", "r") as mIoU_v:
        for value in mIoU_v:
            store_miou_validation = value.split(",")
            store_miou_validation.pop()
    for i in store_miou_validation:
        mIoU_all_val.append(float(i))

    with open("training_L1.txt", "r") as l1:
        for value in l1:
            store_l1 = value.split(",")
            store_l1.pop()
    for i in store_l1:
        l1_all.append(float(i))
    with open("evaluation_L1.txt", "r") as l1_val:
        for value in l1_val:
            store_l1_validation = value.split(",")
            store_l1_validation.pop()
    for i in store_l1_validation:
        l1_all_val.append(float(i))

    fig, ax = plt.subplots()
    ax.plot(range(0, len(mIoU_background_all)), mIoU_background_all, color='red', label="training")
    ax.plot(range(0, len(mIoU_background_all_val)), mIoU_background_all_val, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("mIoU")
    plt.title("mIoU - background")
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(range(0, len(mIoU_car_all)), mIoU_car_all, color='red', label="training")
    ax.plot(range(0, len(mIoU_car_all_val)), mIoU_car_all_val, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("mIoU")
    plt.title("mIoU - car")
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(range(0, len(mIoU_junction_all)), mIoU_junction_all, color='red', label="training")
    ax.plot(range(0, len(mIoU_junction_all_val)), mIoU_junction_all_val, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("mIoU")
    plt.title("mIoU - junction")
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(range(0, len(mIoU_road_all)), mIoU_road_all, color='red', label="training")
    ax.plot(range(0, len(mIoU_road_all_val)), mIoU_road_all_val, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("mIoU")
    plt.title("mIoU - road")
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(range(0, len(mIoU_all)), mIoU_all, color='red', label="training")
    ax.plot(range(0, len(mIoU_all_val)), mIoU_all_val, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("mIoU")
    plt.title("comparison of differnt mIoU")
    plt.show()

    fig, ax = plt.subplots()
    ax.plot(range(0, len(l1_all)), l1_all, color='red', label="training")
    ax.plot(range(0, len(l1_all_val)), l1_all_val, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("l1 regression loss")
    plt.title("comparison of different line regression loss")
    plt.show()


plot()