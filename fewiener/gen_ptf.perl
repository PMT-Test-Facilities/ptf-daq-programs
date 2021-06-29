#!/usr/bin/perl -w

my $confOdb = "odbedit";
my $confEq = "/Equipment/PtfWiener";

my %names  = readOdbArray("$confEq/Settings/Names");
my %switch = readOdbArray("$confEq/Variables/Switch");
my %hwnames = readOdbArray("$confEq/Status/HWNames");

#print "length: ", scalar %names, " and ", scalar %switch;

my %xgroups;

#undef %names;
#$names{1} = "A%B%C1%D1";
#$names{2} = "A%B%C1%D2";
#$names{3} = "A%B%C2%D1";
#$names{4} = "A%B%C2%D2";

if (0) {
  test_xcmp("A", "A");
  test_xcmp("ABC", "ABC");
  test_xcmp("AA", "ZZ");
  test_xcmp("ZZ", "Z");
  test_xcmp("U1", "U1");
  test_xcmp("U1", "U2");
  test_xcmp("U10", "U2");
  test_xcmp("U10", "U11");
  test_xcmp("U22A", "U22B");
  test_xcmp("U22A100", "U22B10");
  die "Here!";
}

my $maxDepth = 0;

foreach my $k (keys %names)
  {
    my $name = $names{$k};

    my @n = split(/\%/, $name);

    my $depth = scalar @n;
    $maxDepth = $depth     if ($depth > $maxDepth);

    my $xg = "";
    while (1)
      {
	my $n = shift @n;
	last if ! defined $n;

	my $xxg = $xg;

	$xg .= "%" if length $xg>0;

	my $xr = "$xg$n";

	if (defined $n[0])
	  {
	    #print "-[$n] [$xxg] -> [$xr]\n";
	    $xgroups{$xxg}{$xr} = 1;
	  }
	else
	  {
	    #print "-[$n] [$xxg] -> [$name]\n";
	    $xgroups{$xxg}{$k} = 1;
	  }

	$xg .= $n;
      }
  }

if (0)
  {
    foreach my $g (sort keys %xgroups)
      {
	print "$g: ";
	my @gg = sort keys %{$xgroups{$g}};
	print scalar @gg;
	foreach my $gg (@gg)
	  {
	    print " $gg ";
	  }
	
	my $cnt = countGroup($g);
	print ", Count $cnt";
	print "\n";
      }
  }

#die "Here!";

my $table = "\n";
my $style = "\n";
my $script = "\n";
my %script;
my $ig = 0;

$style .= "\n";
$style .= "td.style_mainSwitch { background-color: <odb src=\"$confEq/Status/mainSwitchColor\"> }\n";
$style .= "td.style_mainStatus { background-color: <odb src=\"$confEq/Status/mainStatusColor\"> }\n";
$style .= "td.style_delayedMainStatus { background-color: <odb src=\"$confEq/Status/delayedMainStatusColor\"> }\n";
$style .= "\n";

if (1)
  {
    $table .= "<table border=1 cellpadding=2>\n";

    $table .= "<tr>\n";
    $table .= "<th colspan=$maxDepth>Name<th>HWCH<th>Action<th>switch<th>demand<th>measured<th>current<th>sparks<th>status\n";
    $table .= "</tr>\n";

    my $g = "";

    my @gg = sort { xcmp($a, $b); } keys %{$xgroups{$g}};

    $scriptIndex{$g} = $ig;
    $ig++;

    foreach my $gg (@gg)
      {
	outputGroup("", $gg, $maxDepth);

	my $ii = $scriptIndex{$gg};
	$script{$g} .= "  setSwitch_$ii(value); // group $gg\n" if defined $ii;
      }

    $table .= "</table>\n";
  }


if (1)
  {
    foreach my $k (sort keys %script)
      {
	#print "[$k]\n";
	my $ik = $scriptIndex{$k};
	$script .= "function setSwitch_$ik(value) { // group $k\n";
	$script .= "  clearTimeout(reloadTimerId);\n";

	$script .= $script{$k};
	$script .= "}\n\n";
      }

    $script .= "function setSwitch(value) { // top level\n";
    $script .= "  setSwitch_0(value);\n";
    $script .= "}\n\n";
  }

my $webpage = <<EOF
<html>
  <head>
    <title>HV control</title>
    <!-- <meta http-equiv="Refresh" content="5">   -->

  </head>

  <body>
    <form name="form1" method="Get">
      <table border=1 cellpadding=2 width="100%">
	<tr><th colspan=2>HV control</tr>
	<tr><td width="50%">
            <input value="Status"    name="cmd" type="submit">
            <input value="ODB"       name="cmd" type="submit">
            <input value="History"   name="cmd" type="submit">
          </td><td>
            <center> <a href="http://midas.triumf.ca"> >>>> Midas Help <<<< </a></center>
          </td></tr>
      </tr>
      </table>
    </form>

    <script src='mhttpd.js'></script>
    <script src='obsolete.js'></script>

    <script>
      var reloadTimerId = 0;
    </script>

    <script>
      $script
    </script>

    <style type="text/css">
      $style
    </style>

    <form name="form2" method="Get">
      <hr>
      <table border=1 cellpadding=2">
        <tr>
          <td class=style_mainSwitch>ISEG/MPOD Main switch: <odb src="$confEq/Settings/mainSwitch"> / <odb src="$confEq/Readback/sysMainSwitch.0"></td>
          <td class=xstyle_mainStatus>Output Enabled: <odb src="$confEq/Settings/outputEnable"></td>
          <td class=style_mainStatus>Status: <odb src="$confEq/Status/mainStatus"></td>
          <td class=style_delayedMainStatus> Delayed: <odb src="$confEq/Status/delayedMainStatus"></td>
          <td>Control enabled: <odb src="$confEq/Settings/EnableControl"></td>
          <!--- <td><input type=button value="turn on and ramp up" onClick="setSwitch(1); window.location.reload();"></input></td> --->
          <!--- <td><input type=button value="ramp down and turn off" onClick="setSwitch(0); window.location.reload();"></input></td> --->
          <!--- <td>Set voltages: <input type=input size=7 value="0" onKeyPress="if (event.keyCode==13) { setVoltage(this.value); window.location.reload() }"></input></td> --->
        </tr>
        <tr>
	  <td>
             <input type=button value=\"Main switch ON\" onClick=\"clearTimeout(reloadTimerId); ODBSet('$confEq/Settings/mainSwitch', 1); window.location.reload();\"></input>
             <input type=button value=\"Main switch OFF\" onClick=\"clearTimeout(reloadTimerId); ODBSet('$confEq/Settings/mainSwitch', 0); window.location.reload();\"></input>
          </td>
	  <td>
             <input type=button value=\"Output ON\" onClick=\"clearTimeout(reloadTimerId); ODBSet('$confEq/Settings/outputEnable', 1); window.location.reload();\"></input>
             <input type=button value=\"Output OFF\" onClick=\"clearTimeout(reloadTimerId); ODBSet('$confEq/Settings/outputEnable', 0); window.location.reload();\"></input>
          </td>
        </tr>
      </table>
      <hr>
      $table
    </form>

    <script>
      //alert("Hello!");
      reloadTimerId = setTimeout('window.location.reload()', 10000);
    </script>

  </body>
</html>
EOF
;

open(OUT, ">hv_ptf.html");
print OUT $webpage;
close OUT;

exit 0;

sub countGroup
  {
    my $g = shift @_;

    my @gg = sort keys %{$xgroups{$g}};
    my $nn = scalar @gg;

    my $count = 1;

    if ($nn > 0)
      {
	foreach my $gg (@gg)
	  {
	    $count += countGroup($gg);
	  }

	return $count;
      }
    else
      {
	return 1;
      }
  }

sub outputGroup
  {
    my $top = shift @_;
    my $g   = shift @_;
    my $depth = shift @_;

    my @gg = sort { xcmp($a, $b); } keys %{$xgroups{$g}};
    #my $nn = $xgroups{$g};  #scalar @gg;

    my $nn = countGroup($g);

    if (! defined $names{$g})
      {
	#print "group $g: $nn\n";

	my $n1 = $nn;


	my $tr = "";
	$tr .= "<tr>\n";
	$tr .= "<td rowspan=$n1 align=center>$g";
	$tr .= " ";
	$tr .= "<br>";
	$tr .= "<input type=button value=\"on\" onClick=\"setSwitch_$ig(1); window.location.reload();\"></input>";
	$tr .= "<input type=button value=\"off\" onClick=\"setSwitch_$ig(0); window.location.reload();\"></input>";
        $tr .= "\n";
        $tr .= "</tr>\n";
        $tr .= "\n";

	$table .= $tr;

	$scriptIndex{$g} = $ig;
	$ig++;

	foreach my $gg (@gg)
	  {
	    outputGroup($g, $gg, $depth-1);

	    my $ii = $scriptIndex{$gg};
	    $script{$g} .= "  setSwitch_$ii(value); // group $gg\n" if (defined $ii);
	  }
      }
    else
      {
	my $k = $g;
	my $e = $names{$k};
        my $s = $switch{$k};

        return if !defined $s;

	my $tr = "";
	$tr .= "<tr class=style_$k >\n";
	$tr .= "<td colspan=$depth align=left>$e</td>\n";
	$tr .= "<td align=left>$hwnames{$k} [$k]</td>\n";
	$tr .= "<td align=left><input type=button value=\"on\" onClick=\"clearTimeout(reloadTimerId); ODBSet('$confEq/Settings/outputSwitch[$k]', 1); window.location.reload();\"></input><input type=button value=\"off\" onClick=\";clearTimeout(reloadTimerId); ODBSet('$confEq/Settings/outputSwitch[$k]', 0); window.location.reload();\"></input></td>\n";
	$tr .= "<td class=style_switch_$k align=center><odb src=\"$confEq/Settings/outputSwitch[$k]\"> / <odb src=\"$confEq/Variables/switch[$k]\"></td>\n";
	$tr .= "<td class=style_demand_$k align=center><odb src=\"$confEq/Settings/outputVoltage[$k]\" edit=1> / <odb src=\"$confEq/Variables/demandVoltage[$k]\" format=\"%.1f\"></td>\n";
        $tr .= "<td class=style_measured_$k align=center><odb src=\"$confEq/Variables/senseVoltage[$k]\" format=\"%.1f\"> V</td>\n";
        $tr .= "<td align=center><odb src=\"$confEq/Variables/current[$k]\" format=\"%.6f\"> A</td>\n";
        $tr .= "<td align=center><odb src=\"$confEq/Variables/sparkCount[$k]\"></td>\n";
	$tr .= "<td class=style_status_$k align=center><odb format=0x%x src=\"$confEq/Variables/status[$k]\">\n";
	$tr .= "(<odb src=\"$confEq/Readback/outputStatusString[$k]\">)</td>\n";
	$tr .= "</tr>\n";
	$tr .= "\n";

	$table .= $tr;

	$style .= "tr.style_$k { background-color: <odb src=\"$confEq/Status/outputColor[$k]\"> }\n";
	$style .= "td.style_switch_$k { background-color: <odb src=\"$confEq/Status/switchColor[$k]\"> }\n";
	$style .= "td.style_status_$k { background-color: <odb src=\"$confEq/Status/statusColor[$k]\"> }\n";
	$style .= "td.style_demand_$k { background-color: <odb src=\"$confEq/Status/demandColor[$k]\"> }\n";
	$style .= "td.style_measured_$k { background-color: <odb src=\"$confEq/Status/measuredColor[$k]\"> }\n";

	$script{$top} .= "  ODBSet(\'$confEq/Settings/outputSwitch[$k]\', value);\n";
      }
  }

sub xcmp
  {
    my $a = shift @_;
    my $b = shift @_;
    my $aa = $names{$a};
    $aa = $a if ! defined $aa;
    my $bb = $names{$b};
    $bb = $b if ! defined $bb;
    my $v = $aa cmp $bb;
    #print "compare $a=[$aa] $b=[$bb] $v\n";
    return $v if $v == 0;

    while (1) {
      #print "compare [$aa] [$bb]\n";

      return -1 if (length($aa)==0);
      return +1 if (length($bb)==0);

      $aa =~ /^(\d+)(.*)$/;
      my $a1 = $1;
      my $a2 = $2;
      $bb =~ /^(\d+)(.*)$/;
      my $b1 = $1;
      my $b2 = $2;

      if (defined $a1 && defined $b1) {
	my $ai = int($a1);
	my $bi = int($b1);
	$v = $a1 <=> $bi;
	#print "compare numbers [$a1] $ai [$b1] $bi gives $v\n";

	return $v if $v != 0;

	$aa = $a2;
	$bb = $b2;
	next;
      }

      my $ca = substr($aa, 0, 1);
      $aa = substr($aa, 1);

      my $cb = substr($bb, 0, 1);
      $bb = substr($bb, 1);

      $v = $ca cmp $cb;

      return $v if ($v != 0);
    }

    #return $v;
    die "Here!";
  }

sub test_xcmp
    {
      my $a = shift @_;
      my $b = shift @_;
      my $v = xcmp($a, $b);
      print "compare [$a] [$b] gives $v\n";
    }

sub readOdbArray
  {
    my $path = shift @_;
    my %a;

    open(IN, "$confOdb -c \"ls -l $path\" |");
    while (my $in = <IN>)
      {
	my ($i, $v) = $in =~ /\[(\d+)\]\s+(.*)\s+$/;
	#print "[$i] [$v] $in\n";
	next unless defined($i);
	next unless length($i)>0;
	$i = int($i);
	$a{$i} = $v;
      }
    close IN;

    return %a;
  }

# end
