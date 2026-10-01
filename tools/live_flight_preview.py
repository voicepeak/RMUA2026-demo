#!/usr/bin/env python3
"""Show the live recorded camera of an offscreen race without Vulkan UI."""
import argparse
import json
from pathlib import Path
import time
import tkinter as tk
from PIL import Image,ImageTk


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args()
    flight=args.run/'flight'
    root=tk.Tk()
    root.title('RMUA seed123 — 实时飞行预览')
    root.geometry('1000x810+280+120')
    root.configure(background='#18202b')
    status=tk.StringVar(value='等待实例起飞与相机画面…')
    tk.Label(root,textvariable=status,background='#18202b',foreground='white',font=('Sans',15)).pack(pady=12)
    canvas=tk.Label(root,background='black')
    canvas.pack(fill='both',expand=True,padx=10)
    view=tk.StringVar(value='left')
    bar=tk.Frame(root,background='#18202b')
    bar.pack(fill='x',pady=8)
    for label,value in [('前视相机','left'),('识别画面','det')]:
        tk.Radiobutton(bar,text=label,value=value,variable=view,background='#18202b',
                       foreground='white',selectcolor='#27364a',activebackground='#27364a').pack(side='left',padx=12)
    tk.Label(bar,text='seed123 · 相机约 2 帧/秒 · 关闭窗口只关闭预览',
             background='#18202b',foreground='#c3cbd5').pack(side='right',padx=12)
    offset=0
    pending=''
    latest=None
    displayed=None

    def update():
        nonlocal offset,pending,latest,displayed
        stream=flight/'streams.jsonl'
        if stream.exists():
            with stream.open() as file:
                file.seek(offset)
                pending+=file.read()
                offset=file.tell()
            lines=pending.split('\n')
            pending=lines.pop()
            for line in lines:
                try:row=json.loads(line)
                except ValueError:continue
                if row.get('topic')=='telemetry':latest=row
        if latest:
            data=latest['data']
            if time.time()-latest['received']<3.:
                status.set('实测速度 %.1f m/s    路线进度 %.0f m'%
                           (data.get('measured_progress_speed',0.),data.get('s',0.)))
            else:status.set('飞行数据已停止更新')
        frame=max(flight.glob(view.get()+'_*.jpg'),default=None)
        if frame is not None and frame!=displayed:
            try:
                with Image.open(frame) as raw:
                    picture=raw.convert('RGB')
                    picture.thumbnail((max(320,canvas.winfo_width()),max(240,canvas.winfo_height())))
                photo=ImageTk.PhotoImage(picture)
                canvas.configure(image=photo)
                canvas.image=photo
                displayed=frame
            except (OSError,ValueError):pass  # Recorder may still be writing.
        root.after(200,update)

    root.after(200,update)
    root.lift()
    root.mainloop()


if __name__=='__main__':main()
