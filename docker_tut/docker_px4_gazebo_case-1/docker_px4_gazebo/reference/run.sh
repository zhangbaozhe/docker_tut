docker compose -f compose-w64-plan.yaml up &
sleep 8h && 
cd data-plan && ls | grep num | zip data_0223_low_noise_plan.zip -@ &&
cd .. &&
docker compose -f compose-w64-plan.yaml down && 
docker compose -f compose-w64-point.yaml up &
sleep 5h &&
cd data-point && ls | grep num | zip data_0223_low_noise_point.zip -@ &&
cd .. &&
docker compose -f compose-w64-point.yaml down  
