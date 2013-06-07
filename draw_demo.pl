#!/opt/local/bin/perl
$| = 1;
use strict;

# do not edit while process.pl is running #
# or emacs auto-save will make process.pl #
# try to process itself                   #

my($CONVERT) = "/opt/local/bin/convert";
my($INDIR) = "./pictures";
#my($BUILDDIR) = "/Users/dgold/dev/m464draw/m464q/gfx";
my($DOKERMITDIR) = "../../tools";
my($BACKUPDIR) = "./unprocessed";
unless(-e $BACKUPDIR) {
  mkdir($BACKUPDIR) || die("failure creating backup dir $BACKUPDIR");
}

# number of images at start of wad that we won't ever replace
# add 1 for the script itself.  ie: 4 permanent images -> set to 5
# or: divide drawdemo.wad byte size by 8192 to get image count, then
# subtract from the end the number of replaceable images.
my($STARTOFFSET) = 24;

# number of images to rotate through
# script and wad file must be pre-loaded with this number of drawings
my($NUMPIX) = 6;

# which image was the last one we replaced.
my($LASTPIC) = 0;  

sub main() {

    if($ARGV[0]) {  # force processing of just the named image
	my($thing) = $ARGV[0];
	print("### processing $thing ###\n");
	    #rename("$INDIR/$thing", "$INDIR/$lcthing") || die("failure renaming $thing");
	    imageMagick($thing);
	exit(0);
    }
    
    #get initial directory length value
    opendir(IWD, $INDIR) || die "can't open $INDIR";
    my(@dirContents) = readdir(IWD);
    closedir(IWD);

    #populate hash table
    my(%listOfImages);
    my($time) = time();
    my($item, $thing, $imgcount);
    foreach $item (@dirContents) {
	$listOfImages{$item} = $time;
    }
    print "waiting for image\n";
    
    while(1) {
	# read directory
	opendir(IWD, $INDIR) || die "can't open $INDIR";
	my(@dirContents) = readdir(IWD);
	closedir(IWD);
	sleep(3);
	
	#find out which one is the new file
        $imgcount = 0;
       	foreach $thing (@dirContents) {
	    chomp($thing);
	    unless($thing =~ /\.(jpg|gif|png|jpeg)$/i) {
		next;
	    }
            $imgcount++;
	    unless ($listOfImages{"$thing"}) {
		print "\n### found new image: $thing ###\n\n";
		my($fname);
		if($thing =~ /draw\d+\.jpg/) {
                  $fname = $thing;
		} else {
		  $fname  = "draw" . time() . ".jpg";
		}
		rename("$INDIR/$thing", "$INDIR/$fname") || die("failure renaming $thing");
		$listOfImages{$fname} = $time; #add this file to the hash
		
		imageMagick("$fname");
		print("### image done ###\n");
	    }
	}
        print("total images: $imgcount\n");
    }
}

sub imageMagick($) {
    my($filenameraw) = shift;
    my($filename) = "$INDIR/$filenameraw";
    $filenameraw =~ /\./;
    my($filebase) = $`; # use just the title, no extension, to meet makescript format
    print("processing $filename\n");
    
    system("cp -n $filename $BACKUPDIR");  #make a backup of the image
    if($?) {
	print("### FAILURE copying $filename\n");
	return;
    }
    
    #modify the image for public consumption
    system("$CONVERT $filename -verbose -normalize $filename");
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    system("$CONVERT $filename -verbose -fuzz 50% -trim $filename");
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    system("$CONVERT $filename -verbose -fuzz 15% -fill \#000000 -opaque \#ffffff $filename");
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    system("$CONVERT $filename -verbose -modulate 80,150 $filename");
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    system("$CONVERT $filename -verbose -shave 10x10 $filename");
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    
    # we will link $BUILDDIR/pictures to 
    #system("cp $filename $BUILDDIR/pictures");
    #if($?) {
    #	print("### FAILURE ###\n");
    #	    return;
    #}
    #system("cd $BUILDDIR; make");
    system("make");
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    
    #`cd $BUILDDIR; /opt/local/bin/perl automakescript $filebase test2.script test2.wad`; #add image to playlist & compile out wad
    
    my($imgoffset) = 8192 * ($STARTOFFSET + $LASTPIC);
    #print("dokermit ...$filebase.img $imgoffset $LASTPIC\n");
    #system("cd $DOKERMITDIR; ./dokermit ../m464q/gfx/img/$filebase.img $imgoffset");
    print("dokermit ../m464q/gfx/img/$filebase.img ${imgoffset}l   ($LASTPIC)\n");
    system("cd $DOKERMITDIR; ./dokermit ../m464q/gfx/img/$filebase.img ${imgoffset}l");
    $LASTPIC = ($LASTPIC + 1) % $NUMPIX;
   
    if($?) {
	print("### FAILURE ###\n");
	    return;
    }
    
}

# imageMagick command v1:
#    `$CONVERT $filename -normalize -fuzz 20% -fill \#000000 -opaque \#ffffff -colors 16 -resize 300x300 -rotate 90 $filename`;


main();

