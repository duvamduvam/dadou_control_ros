import tkinter as tk
from tkinter import ttk

root = tk.Tk()
root.title('Pack Demo')
root.geometry("300x200")

# place widgets top down
top = tk.Frame(
    root,
    bg="red",
)

top.pack(
    fill='x',
    side='top'
)

glove_label = tk.Label(
    top,
    text='Glove',
    bg="blue",
)

glove_label.pack(
    ipadx=10,
    ipady=10,
    fill='x',
    side='left'
)

sequence_label = tk.Label(
    top,
    text='Sequence',
    bg="pink",
)

sequence_label.pack(
    ipadx=10,
    ipady=10,
    fill='x',
    side='left'
)

option_label = tk.Label(
    top,
    text='Options',
    bg="yellow",
)

option_label.pack(
    ipadx=10,
    ipady=10,
    fill='x',
    side='left'
)

left = tk.Frame(
    root,
    bg="orange",
)

left.pack(
    ipadx=100,
    fill='y',
    side='left'
)

main = tk.Label(
    root,
    bg='#116562',
)

main.pack(
    fill='both',
    expand=True,
    side='right'
)

#main.get

canvas = tk.Canvas(main)
canvas.create_line(15, main.winfo_reqheight(), 200, 25)
canvas.create_line(300, 35, 300, main.winfo_height(), dash=(4, 2))
canvas.create_line(55, 85, 155, 85, 105, 180, 55, 85)

canvas.create_oval(5, main.winfo_height(), 20, 20)

canvas.pack(fill='both', expand=1)


root.mainloop()