Guide to contributing to the Arduino Floppy Disk Controller
===========================================================

The Arduino firmware is already very useful und can read everything.
The track is always read at-once, because the sector-wise reading depends
on the format (AMIGA, PC, ATARI, COMMODORE etc..). Letting the format be 
detected by PC software is part of the concept, and sufficiently smart, 
so this need not be changed. To read a single sector, the PC software can 
instruct the Arduino formware to read the whole track, and then it can 
decode only a single sector. There are options to wait or not wait for the 
index pulse when reading, so that even a fast reading should be possible.

Improvements can be done with writing to the disk. Here contributions are 
welcome.

However writing is olnly supported for DD track-at once currently. Since the 
Arduino cannot buffer the data for a whole track, the writing need do be 
in sync with data arriving via the serial interface. This is a bit tricky 
and does not always work. Writing a track at once is the normal way to either
* format the disk (ATARI ST, PC) or
* write data to it (AMIGA).
Writing individual Sectors should maybe made possible in furture (and a single 
sector data can well be buffered by the firmware). But sector detection is 
format dependant. Luckily for the ATARI ST and PC disks the sector marks 
are the same. COMMODORE disks will maybe later be supported. 


Other ideas: 
* How about a block device driver for linux (interfacinge the Arduino firmware)?

## License and attribution

All contributions must be properly licensed and attributed. If you are contributing your own original work, then you are offering it under a CC-BY license (Creative Commons Attribution). If it is code, you are offering it under the GPL-v2. You are responsible for adding your own name or pseudonym in the Acknowledgments file, as attribution for your contribution.

If you are sourcing a contribution from somewhere else, it must carry a compatible license. The project was initially released under the GNU public licence GPL-v2 which means that contributions must be licensed under open licenses such as MIT, CC0, CC-BY, etc. You need to indicate the original source and original license, by including a comment above your contribution. 

## Contributing with a Pull Request

The best way to contribute to this project is by making a pull request:

1. Login with your Github account or create one now
2. [Fork](https://github.com/kollokollo/ArduinoFloppyDiskReader#fork-destination-box) the ArduinoFloppyDiskReader repository. Work on your fork.
3. Create a new branch on which to make your change, e.g.
`git checkout -b my_code_contribution`, or make the change on the `new` branch.
4. Edit the file where you want to make a change or create a new file in the `contrib` directory if you're not sure where your contribution might fit.
5. Edit `ACKNOWLEGEMENTS` and add your own name to the list of contributors under the section with the current year. Use your name, or a github ID, or a pseudonym.
6. Commit your change. Include a commit message describing the correction.
7. Submit a pull request against the ArduinoFloppyDiskReader repository.


## Contributing with an Issue

If you find a mistake and you're not sure how to fix it, or you don't know how to do a pull request, then you can file an Issue. Filing an Issue will help us see the problem and fix it.

Create a [new Issue](https://github.com/kollokollo/ArduinoFloppyDiskReader/issues/new) now!


## Thanks

We are very grateful for your support. With your help, this implementation will be a great project. 
