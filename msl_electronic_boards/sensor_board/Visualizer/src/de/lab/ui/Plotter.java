package de.lab.ui;

import java.awt.Color;
import java.awt.Component;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.Icon;

public class Plotter implements Icon {

	private ArrayList<Integer> data;
	
	public Plotter()
	{
		this.data = new ArrayList<Integer>();
	}
	
	@Override
	public int getIconHeight() {
		// TODO Auto-generated method stub
		return 150;
	}

	@Override
	public int getIconWidth() {
		// TODO Auto-generated method stub
		return 520;
	}

	@Override
	public void paintIcon(Component arg0, Graphics arg1, int arg2, int arg3) {
		// TODO Auto-generated method stub
		Color color = arg1.getColor();
		arg1.setColor(new Color(255,0,0));
		System.out.println(this.data.size());
		for(int i = 0; i < this.data.size()-1; i++)
		{
			//System.out.println("Writing data: " + this.data.get(i));
			//arg1.fillOval(26*i, arg0.getHeight()-this.data.get(i), 5, 5);
			arg1.drawLine(26*i, arg0.getHeight()-this.data.get(i), 26*(i+1), arg0.getHeight()-this.data.get(i+1));
		}
		
		arg1.setColor(color);
	}

	public ArrayList<Integer> getData() {
		return data;
	}

	public void setData(ArrayList<Integer> data) {
		this.data = data;
	}

}
