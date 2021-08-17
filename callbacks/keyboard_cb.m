function keyboard_cb(src,event)
   switch event.Key
       case 'w'
          current_campos = campos;
          campos(current_campos+[0 0 0.5]);
       case 's'
           current_campos = campos;
          campos(current_campos-[0 0 0.5]);
   end
end