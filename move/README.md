# What are all of these? 

I'm not really sure! It wasn't documented. But, here are some guesses

 - `feMove`: the main one
 - `feMove_tilt`: this version of feMove included the tilting and rotations. 
 - `feMove_no_tilt`: Vincent committed this as a version of feMove which doesn't have tilting or rotations. It was exactly identical to feMove until I re-implemented those tilts/rotations. 
 - `feMove_new_collision`: seems like it might be related to Elena's work? Unsure. This uses the **old midas** format. 
 - `feMove_midas_changed`: ????
 - `feMove_no_tilt_precedent`: ????
 - `feMove_Reduced`: ????
 

 To my understanding, most of these were never committed. 
 Instead whoever was running scans would copy an feMove from above and paste it as feMove.cxx 

 That way they could, presumably, swap out feature sets. s