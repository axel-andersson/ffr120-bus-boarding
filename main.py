def main():
    print("hello world")

def plot_crowd():
    print("Dummy fn for plotting crowds.")

if __name__ == "__main__":
    main()

def apply_attraction(x,y,x_dest, y_dest, w_at):

    F_at_x = x_dest - x
    F_at_y = y_dest - y

    return F_at_x*w_at, F_at_y*w_at


