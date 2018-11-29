#!/usr/bin/env python

import npyscreen

global lname

class nsteps_slider(npyscreen.TitleSlider):
	def when_value_edited(self):
		global lname
		lname.value = str(self.value)

class form_object(npyscreen.ActionForm, npyscreen.FormWithMenus):
	def create(self):
		global lname
		lname = self.add(npyscreen.TitleText, name = "First Name:",  value = "LOL")
		self.add(npyscreen.TitleText, name = "Last name:", w_id=None, max_height=None, rely=None, relx=None)
		self.nstep_slider = self.add(nsteps_slider, name = "n steps", value = 20, out_of = 40, step = 0.1)

		self.menu = self.new_menu(name="Main Menu", shortcut='m ')
		self.menu.addItem("Item 1", self.press_1, "1")
		self.menu.addItem("Item 2", self.press_2, "2")
		self.menu.addItem("Exit form", self.exit_form, "^X")

		self.submenu = self.menu.addNewSubmenu("A sub menu", 's')
		# self.add(npyscreen.TitleMultiSelect, name = "Which one?", value = ["WOW", "NO", "LOL"])		 

	def press_1(self):
		npyscreen.notify_confirm("You pressed item 1", title="Item 1")

	def press_2(self):
		npyscreen.notify_confirm("You pressed item 2", title="Item 2")

	def exit_form(self):
		self.parentApp.switchForm(None)

	def afterEditing(self):
		# self.parentApp.setNextForm(None)
		pass

	def on_ok(self):
		npyscreen.notify_confirm("Form has been saved", title="Yay!", form_color='STANDOUT', wrap=True, wide=True)
		self.parentApp.setNextForm(None)

	def on_cancel(self):
		exiting = npyscreen.notify_yes_no("Are you sure?", title="Huh?", editw=1)
		if(exiting):
			npyscreen.notify_confirm("OK. Form  not saved.", title="Boi!", editw=1)
			self.parentApp.setNextForm(None)

		else:
			npyscreen.notify_confirm("Kelsa nodkond hogu", title="Aayta", form_color='STANDOUT', wrap=True, wide=False, editw=1)

class App(npyscreen.NPSAppManaged):
	def onStart(self):
		self.addForm('MAIN', form_object, name = "NPS Form")


def simp_func(*args):
	form = npyscreen.Form(name = 'BIG LOL!')
	form.edit()

if __name__ == "__main__":
	app = App().run()
