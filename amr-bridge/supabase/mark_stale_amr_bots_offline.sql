-- Applied on project AMR (khucmrlbstdlbxcuboub). Kept here for repo history.
-- Flips amr_bots.online to false when last_seen_at is older than 20s.
-- Scheduled via pg_cron every 15 seconds (job name: mark-stale-amr-bots-offline).

create extension if not exists pg_cron with schema pg_catalog;

create or replace function public.mark_stale_amr_bots_offline(
  p_stale_after interval default interval '20 seconds'
)
returns integer
language plpgsql
security definer
set search_path to 'public'
as $$
declare
  n integer;
begin
  update public.amr_bots
     set online = false,
         updated_at = now()
   where online is true
     and (last_seen_at is null or last_seen_at < now() - p_stale_after);
  get diagnostics n = row_count;
  return n;
end;
$$;

revoke all on function public.mark_stale_amr_bots_offline(interval)
  from public, anon, authenticated;
grant execute on function public.mark_stale_amr_bots_offline(interval) to service_role;
