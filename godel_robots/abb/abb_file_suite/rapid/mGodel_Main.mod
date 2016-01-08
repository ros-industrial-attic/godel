MODULE mGodel_DemoMain
    PROC Godel_Main()
        !Delete Files if they exist
        IF IsFile("HOME:/PARTMODULES/mGodelBlend.mod") RemoveFile "HOME:/PARTMODULES/mGodelBlend.mod";
        IF ModExist("mGodel_Blend") EraseModule("mGodel_Blend");
        
        WHILE true DO
          !Wait for Blend File
          WaitUntil IsFile("HOME:/PARTMODULES/mGodelBlend.mod");
          WaitTime 0.25;
          Load "HOME:/PartModules" \File:="mGodelBlend.MOD";
          %"Godel_Blend"%;
          UnLoad "HOME:/PartModules" \File:="mGodelBlend.MOD";
          RemoveFile "HOME:/PARTMODULES/mGodelBlend.mod";
        ENDWHILE
        !
    ENDPROC
    
ENDMODULE