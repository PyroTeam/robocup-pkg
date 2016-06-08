

# Echo color function
function echoC
{
	START="\e[0m"
	END="\e[0m"

	if [ "$#" -eq "2" ];then
		color=$1
		str=$2
	else
		str=$1
		color="NULL"
	fi

	case "$color" in
	"r" | "red" | "RED")
	    START="\e[31m"
	    ;
	"g" | "green" | "GREEN")
	    START="\e[32m"
	    ;
	"o" | "orange" | "ORANGE")
	    START="\e[33m"
	    ;
	"b" | "blue" | "BLUE")
	    START="\e[34m"
	    ;
	"p" | "purple" | "PURPLE")
	    START="\e[35m"
	    ;
	"lb" | "lightblue" | "LIGHTBLUE")
	    START="\e[36m"
	    ;
	*)
		START=$END
		;
	esac
	echo -e "$START$str$END"
}