import matplotlib.pyplot as plt
import csv
import mplcursors
import time
from matplotlib.widgets import Button

def read_csv(file_path):
    x = []
    y = []
    time_data = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)  # Skip header
        for row in csv_reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
            time_data.append(float(row[2]))  # Append the third column to `time_data`
    return x, y, time_data

def plot_data(x, y, time_data):
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.3)  # Adjust layout to make room for the buttons
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)

    line, = ax.plot(x, y, marker='', linestyle='-', color='r')
    marker_line, = ax.plot(x, y, marker='o', linestyle='', color='k')

    ax.set_title('Map of the Maze')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_xticks([-7.2, -6.6, -6.0, -5.4, -4.8, -4.2, -3.6, -3.0, -2.4, -1.8, -1.2, -0.6, 0, 0.6, 1.2, 1.8, 2.4, 3.0, 3.6, 4.2, 4.8, 5.4, 6.0, 6.6, 7.2])
    ax.set_yticks([-7.2, -6.6, -6.0, -5.4, -4.8, -4.2, -3.6, -3.0, -2.4, -1.8, -1.2, -0.6, 0, 0.6, 1.2, 1.8, 2.4, 3.0, 3.6, 4.2, 4.8, 5.4, 6.0, 6.6, 7.2])
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')  # Ensure equal aspect ratio

    # Add the cursor with tooltips
    cursor = mplcursors.cursor(line, hover=True)
    cursor.connect("add", lambda sel: on_hover(sel, x, y, time_data))

    # Create buttons and link them to the flip and rotate functions
    axflip_y = plt.axes([0.7, 0.05, 0.1, 0.075])
    btn_flip_y = Button(axflip_y, 'Top <-> Bottom')
    btn_flip_y.on_clicked(lambda event: flip_y(ax))

    axflip_x = plt.axes([0.81, 0.05, 0.1, 0.075])
    btn_flip_x = Button(axflip_x, 'Left <-> Right')
    btn_flip_x.on_clicked(lambda event: flip_x(ax))

    axrotate = plt.axes([0.59, 0.05, 0.1, 0.075])
    btn_rotate = Button(axrotate, 'Turn back')
    btn_rotate.on_clicked(lambda event: rotate_180(ax))

    # Connect the scroll event to the zoom handler
    fig.canvas.mpl_connect('scroll_event', lambda event: throttled_scroll(event, ax))

    plt.show()

def on_hover(sel, x, y, time_data):
    index = int(sel.index)  # Convert index to integer
    sel.annotation.set_text(
        f"X: {x[index]:.2f}\nY: {y[index]:.2f}\nTime: {time_data[index]:.2f}")
    sel.annotation.xy = (x[index], y[index])  # Update annotation position
    sel.annotation.get_bbox_patch().set_facecolor('red')  # Optional: background color
    sel.annotation.get_bbox_patch().set_alpha(0.7)  # Optional: transparency
    sel.annotation.set_visible(True)  # Make annotation visible
    sel.annotation.get_figure().canvas.draw_idle()  # Update figure

def flip_y(ax):
    """Function to flip the Y-axis."""
    ax.invert_yaxis()  # Invert the Y-axis
    ax.figure.canvas.draw_idle()  # Update the figure

def flip_x(ax):
    """Function to flip the X-axis."""
    ax.invert_xaxis()  # Invert the X-axis
    ax.figure.canvas.draw_idle()  # Update the figure

def rotate_180(ax):
    """Function to rotate the plot 180 degrees (flip both axes)."""
    ax.invert_xaxis()  # Invert the X-axis
    ax.invert_yaxis()  # Invert the Y-axis
    ax.figure.canvas.draw_idle()  # Update the figure

def on_scroll(event, ax):
    scale_factor = 1.1
    if event.button == 'up':  # Zoom in
        scale_factor = 1 / scale_factor
    elif event.button == 'down':  # Zoom out
        scale_factor = scale_factor

    # Zoom logic
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    
    x_center = event.xdata
    y_center = event.ydata
    
    x_range = xlim[1] - xlim[0]
    y_range = ylim[1] - ylim[0]
    
    new_xlim = [x_center - (x_center - xlim[0]) * scale_factor,
                x_center + (xlim[1] - x_center) * scale_factor]
    new_ylim = [y_center - (y_center - ylim[0]) * scale_factor,
                y_center + (ylim[1] - y_center) * scale_factor]
    
    ax.set_xlim(new_xlim)
    ax.set_ylim(new_ylim)
    ax.figure.canvas.draw_idle()  # Efficiently update the figure

last_scroll_time = 0

def throttled_scroll(event, ax):
    global last_scroll_time
    current_time = time.time()
    if current_time - last_scroll_time > 0.05:  # Adjust the time interval as needed
        on_scroll(event, ax)
        last_scroll_time = current_time

if __name__ == '__main__':
    file_path = 'position_data.csv'
    x, y, time_data = read_csv(file_path)
    plot_data(x, y, time_data)
